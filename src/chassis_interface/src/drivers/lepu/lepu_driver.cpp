#include <chassis_interface/chassis_driver.h>
#include <chassis_interface/drivers/lepu/lepu_protocol.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread/shared_mutex.hpp>
#include <cmath>
#include <thread>
#include <atomic>

namespace chassis_interface
{

// ============================================================
// LepuDriver — 乐普 SLAM 3.0 串口底盘驱动
// ============================================================
class LepuDriver : public ChassisDriver
{
public:
  LepuDriver();
  ~LepuDriver() override;

  // ---- ChassisDriver 接口实现 ----
  bool connect(ros::NodeHandle& nh) override;
  void disconnect() override;
  ChassisState readState() override;
  void writeCommand(const ChassisCommand& cmd) override;
  void emergencyStop() override;
  void resetOdom() override;
  void getDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override;
  std::string getDriverName() const override { return "lepu"; }

private:
  void handleMessage(const std::string& msg);
  double encoderDelta(int32_t current, int32_t previous) const;
  void integrateMotion(double delta_center, double delta_yaw, double dt);
  double normalizeAngle(double angle) const;

  // ---- 参数 ----
  std::string port_;
  int baudrate_;
  bool enable_pose_report_;
  double wheel_separation_;
  double wheel_radius_;
  int encoder_ticks_per_rev_;
  double odom_linear_scale_;
  double odom_angular_scale_;
  bool use_encoder_odom_;

  // ---- 串口链路 ----
  mutable std::mutex serial_mutex_;
  std::unique_ptr<LepuSerialLink> serial_;

  // ---- 状态（保护锁） ----
  mutable boost::shared_mutex state_mutex_;
  double odom_x_, odom_y_, odom_yaw_;
  double linear_vel_, angular_vel_;
  int32_t last_left_encoder_, last_right_encoder_;
  bool encoder_initialized_;
  double meters_per_tick_;
  ros::Time last_encoder_time_;

  // 编码器里程计参数
  double encoder_dt_max_;      // 最大时间间隔 (s)
  double encoder_dt_fallback_; // 兜底时间间隔 (s)

  // nav_pose origin
  bool pose_origin_set_ = false;
  double pose_origin_x_ = 0.0, pose_origin_y_ = 0.0, pose_origin_yaw_ = 0.0;

  // ---- 诊断 ----
  std::atomic<int> msg_count_{0};
  std::atomic<int> error_count_{0};
};

LepuDriver::LepuDriver()
  : odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
  , linear_vel_(0.0), angular_vel_(0.0)
  , last_left_encoder_(0), last_right_encoder_(0)
  , encoder_initialized_(false)
  , meters_per_tick_(0.0)
{
}

LepuDriver::~LepuDriver()
{
  disconnect();
}

bool LepuDriver::connect(ros::NodeHandle& nh)
{
  // 读取驱动专属参数
  nh.param<std::string>("port", port_, "/dev/ttyACM0");
  nh.param<int>("baudrate", baudrate_, 115200);
  nh.param<bool>("enable_pose_report", enable_pose_report_, true);
  nh.param<double>("wheel_separation", wheel_separation_, 0.242);
  nh.param<double>("wheel_radius", wheel_radius_, 0.0705);
  nh.param<int>("encoder_ticks_per_rev", encoder_ticks_per_rev_, 16384);
  nh.param<double>("odom_linear_scale", odom_linear_scale_, 1.0);
  nh.param<double>("odom_angular_scale", odom_angular_scale_, 1.0);
  nh.param<bool>("use_encoder_odom", use_encoder_odom_, true);
  nh.param<double>("encoder_dt_max", encoder_dt_max_, 0.5);
  nh.param<double>("encoder_dt_fallback", encoder_dt_fallback_, 0.05);

  meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / std::max(encoder_ticks_per_rev_, 1);

  // 创建串口链路
  serial_.reset(new LepuSerialLink(port_, baudrate_,
    std::bind(&LepuDriver::handleMessage, this, std::placeholders::_1)));

  if (!serial_->open())
  {
    ROS_ERROR_STREAM("[LepuDriver] Failed to open serial port: " << port_);
    return false;
  }

  ROS_INFO_STREAM("[LepuDriver] Opened " << port_ << " @ " << baudrate_
    << " baud, wheel_sep=" << wheel_separation_ << " wheel_rad=" << wheel_radius_);

  // 初始化底盘
  serial_->sendCommand("model:request");
  if (enable_pose_report_)
  {
    serial_->sendCommand("nav:get_pose[open?on]");
  }

  last_encoder_time_ = ros::Time::now();
  return true;
}

void LepuDriver::disconnect()
{
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_)
    {
      try { serial_->sendCommand("app_vel[0,0]"); } catch (...) {}
      serial_->close();
      serial_.reset();
    }
  }
}

ChassisState LepuDriver::readState()
{
  ChassisState state;
  {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    state.x = odom_x_;
    state.y = odom_y_;
    state.yaw = odom_yaw_;
    state.linear_vel = linear_vel_;
    state.angular_vel = angular_vel_;
    state.left_encoder = last_left_encoder_;
    state.right_encoder = last_right_encoder_;
    state.stamp = ros::Time::now();
  }
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    state.is_connected = serial_ && serial_->isOpen();
  }

  if (!state.is_connected)
  {
    state.error_code = 1;
    state.status_msg = "Serial disconnected";
    error_count_++;
  }

  return state;
}

void LepuDriver::writeCommand(const ChassisCommand& cmd)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_ || !serial_->isOpen())
  {
    error_count_++;
    return;
  }

  char buf[64];
  snprintf(buf, sizeof(buf), "app_vel[%.3f,%.3f]", cmd.linear_vel, cmd.angular_vel);
  serial_->sendCommand(buf);
}

void LepuDriver::emergencyStop()
{
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_)
    {
      serial_->sendCommand("app_vel[0,0]");
    }
  }
  {
    boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
    linear_vel_ = 0.0;
    angular_vel_ = 0.0;
  }
}

void LepuDriver::resetOdom()
{
  boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  linear_vel_ = 0.0;
  angular_vel_ = 0.0;
  encoder_initialized_ = false;
  pose_origin_set_ = false;
}

void LepuDriver::getDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  bool connected;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    connected = serial_ && serial_->isOpen();
  }
  if (!connected)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Serial disconnected");
    stat.add("port", port_);
    return;
  }

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  stat.add("port", port_);
  stat.add("msg_count", msg_count_.load());
  stat.add("error_count", error_count_.load());

  boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
  stat.add("odom_x", odom_x_);
  stat.add("odom_y", odom_y_);
  stat.add("odom_yaw", odom_yaw_);
  stat.add("linear_vel", linear_vel_);
  stat.add("angular_vel", angular_vel_);
}

// ============================================================
// 私有实现
// ============================================================
void LepuDriver::handleMessage(const std::string& msg)
{
  msg_count_++;

  // 日志型号信息
  if (msg.find("model:") == 0 || msg.find("hfls_version:") == 0)
  {
    ROS_INFO_STREAM("[LepuDriver] " << msg);
    return;
  }

  // 警告：定位未完成
  if (msg.find("nav:pose:notfound") == 0)
  {
    ROS_WARN_THROTTLE(5.0, "[LepuDriver] Localizing... (nav:pose:notfound)");
    return;
  }

  ros::Time now = ros::Time::now();

  // 解析编码器
  int32_t left_enc, right_enc;
  if (use_encoder_odom_ && parseWheelEncoders(msg, left_enc, right_enc))
  {
    boost::unique_lock<boost::shared_mutex> lock(state_mutex_);

    if (!encoder_initialized_)
    {
      last_left_encoder_ = left_enc;
      last_right_encoder_ = right_enc;
      last_encoder_time_ = now;
      encoder_initialized_ = true;
      ROS_INFO_STREAM("[LepuDriver] Encoder initialized: L=" << left_enc << " R=" << right_enc);
      return;
    }

    double delta_left = encoderDelta(left_enc, last_left_encoder_) * meters_per_tick_;
    double delta_right = encoderDelta(right_enc, last_right_encoder_) * meters_per_tick_;
    last_left_encoder_ = left_enc;
    last_right_encoder_ = right_enc;

    double dt = (now - last_encoder_time_).toSec();
    if (dt <= 0.0 || dt > encoder_dt_max_) dt = encoder_dt_fallback_;

    double delta_center = 0.5 * (delta_left + delta_right) * odom_linear_scale_;
    double delta_yaw = ((delta_right - delta_left) / wheel_separation_) * odom_angular_scale_;

    integrateMotion(delta_center, delta_yaw, dt);
    last_encoder_time_ = now;
  }

  // 解析 base_vel（速度模式里程计）
  double bv, bw;
  if (parseBaseVel(msg, bv, bw))
  {
    boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
    linear_vel_ = bv * odom_linear_scale_;
    angular_vel_ = bw * odom_angular_scale_;
  }

  // 解析 nav_pose（备用，仅在编码器不可用时生效）
  double nx, ny, nyaw;
  if (parseNavPose(msg, nx, ny, nyaw))
  {
    boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
    if (!use_encoder_odom_)
    {
      if (!pose_origin_set_)
      {
        pose_origin_x_ = nx;
        pose_origin_y_ = ny;
        pose_origin_yaw_ = nyaw;
        pose_origin_set_ = true;
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_yaw_ = 0.0;
        return;
      }
      double rel_x = nx - pose_origin_x_;
      double rel_y = ny - pose_origin_y_;
      double cos_yaw = std::cos(pose_origin_yaw_);
      double sin_yaw = std::sin(pose_origin_yaw_);
      odom_x_ = rel_x * cos_yaw + rel_y * sin_yaw;
      odom_y_ = -rel_x * sin_yaw + rel_y * cos_yaw;
      odom_yaw_ = normalizeAngle(nyaw - pose_origin_yaw_);
    }
  }
}

double LepuDriver::encoderDelta(int32_t current, int32_t previous) const
{
  int32_t delta = current - previous;
  int half_rev = encoder_ticks_per_rev_ / 2;
  if (delta > half_rev) delta -= encoder_ticks_per_rev_;
  else if (delta < -half_rev) delta += encoder_ticks_per_rev_;
  return static_cast<double>(delta);
}

void LepuDriver::integrateMotion(double delta_center, double delta_yaw, double dt)
{
  if (std::abs(delta_center) < 1e-9 && std::abs(delta_yaw) < 1e-9)
  {
    linear_vel_ = 0.0;
    angular_vel_ = 0.0;
    return;
  }

  double mid_yaw = odom_yaw_ + 0.5 * delta_yaw;
  odom_x_ += delta_center * std::cos(mid_yaw);
  odom_y_ += delta_center * std::sin(mid_yaw);
  odom_yaw_ = normalizeAngle(odom_yaw_ + delta_yaw);

  dt = std::max(dt, 1e-6);
  linear_vel_ = delta_center / dt;
  angular_vel_ = delta_yaw / dt;
}

double LepuDriver::normalizeAngle(double angle) const
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace chassis_interface

// pluginlib 注册
PLUGINLIB_EXPORT_CLASS(chassis_interface::LepuDriver, chassis_interface::ChassisDriver)
