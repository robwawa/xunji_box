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

  // ---- 参数 ----
  std::string port_;
  int baudrate_;
  bool enable_pose_report_;
  std::string odom_source_;      // "base_vel" (默认) 或 "nav_pose"
  std::string nav_mode_;         // "mapping"(默认) "navi" "remap"
  double odom_linear_scale_;
  double odom_angular_scale_;

  // ---- 串口链路 ----
  mutable std::mutex serial_mutex_;
  std::unique_ptr<LepuSerialLink> serial_;

  // ---- 状态（保护锁） ----
  mutable boost::shared_mutex state_mutex_;
  double odom_x_, odom_y_, odom_yaw_;
  double linear_vel_, angular_vel_;
  // 编码器值：官方协议无编码器上报，固定为 0
  int32_t last_left_encoder_ = 0, last_right_encoder_ = 0;
  ros::Time last_base_vel_time_;
  bool base_vel_initialized_ = false;
  double dt_max_ = 0.5;            // 最大有效时间间隔 (s)

  // nav_pose origin
  bool pose_origin_set_ = false;
  double pose_origin_x_ = 0.0, pose_origin_y_ = 0.0, pose_origin_yaw_ = 0.0;

  // ---- 定时器 ----
  ros::Timer heartbeat_timer_;

  // nav_pose 速度计算
  double last_nav_x_ = 0.0, last_nav_y_ = 0.0, last_nav_yaw_ = 0.0;
  ros::Time last_nav_pose_time_;
  double last_data_ts_ = 0.0;        // 上一帧 nav:time_pose 自带时间戳

  // 低通滤波 — 抑制位置差分噪声
  double vel_lpf_alpha_ = 0.3;       // 滤波系数 (0~1, 越小越平滑)
  double filtered_linear_vel_ = 0.0;
  double filtered_angular_vel_ = 0.0;
  double max_vel_change_ = 0.5;      // m/s², 单周期最大速度变化

  // ---- 诊断 ----
  std::atomic<int> msg_count_{0};
  std::atomic<int> error_count_{0};
};

LepuDriver::LepuDriver()
  : odom_x_(0.0), odom_y_(0.0), odom_yaw_(0.0)
  , linear_vel_(0.0), angular_vel_(0.0)
{
}

LepuDriver::~LepuDriver()
{
  disconnect();
}

bool LepuDriver::connect(ros::NodeHandle& nh)
{
  // 读取驱动专属参数
  nh.param<std::string>("port", port_, "/dev/lepu_chassis");
  nh.param<int>("baudrate", baudrate_, 115200);
  nh.param<bool>("enable_pose_report", enable_pose_report_, true);
  nh.param<std::string>("odom_source", odom_source_, "base_vel");
  nh.param<std::string>("nav_mode", nav_mode_, "mapping");
  nh.param<double>("odom_linear_scale", odom_linear_scale_, 1.0);
  nh.param<double>("odom_angular_scale", odom_angular_scale_, 1.0);
  nh.param<double>("dt_max", dt_max_, 0.5);
  nh.param<double>("vel_lpf_alpha", vel_lpf_alpha_, 0.3);
  nh.param<double>("max_vel_change", max_vel_change_, 0.5);

  // 创建串口链路
  serial_.reset(new LepuSerialLink(port_, baudrate_,
    std::bind(&LepuDriver::handleMessage, this, std::placeholders::_1)));

  if (!serial_->open())
  {
    ROS_ERROR_STREAM("[LepuDriver] Failed to open serial port: " << port_);
    return false;
  }

  ROS_INFO_STREAM("[LepuDriver] Opened " << port_ << " @ " << baudrate_ << " baud");

  // 初始化底盘
  serial_->sendCommand("model:request");
  if (nav_mode_ == "navi")
    serial_->sendCommand("model:navi");
  else if (nav_mode_ == "remap")
    serial_->sendCommand("model:remap");
  else if (nav_mode_ == "mapping")
    serial_->sendCommand("model:mapping");
  ROS_INFO("[LepuDriver] nav_mode=%s", nav_mode_.c_str());

  if (enable_pose_report_)
  {
    serial_->sendCommand("nav:get_pose[open?on]");
  }

  // 心跳保活 — 官方协议要求 5s 一次 (§1 心跳包)
  heartbeat_timer_ = nh.createTimer(ros::Duration(5.0),
      [this](const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_) serial_->sendCommand("keep_connect");
      });

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

  // 超时归零 — 停止后速度衰减到0
  {
    double age = 0.0;
    if (odom_source_ == "base_vel" && base_vel_initialized_)
      age = (ros::Time::now() - last_base_vel_time_).toSec();
    else if (odom_source_ == "nav_pose" && last_nav_pose_time_.isValid())
      age = (ros::Time::now() - last_nav_pose_time_).toSec();
    if (age > dt_max_)
    {
      state.linear_vel = 0.0;
      state.angular_vel = 0.0;
    }
  }

  return state;
}

void LepuDriver::writeCommand(const ChassisCommand& cmd)
{
  // 连续零速不重复发送
  static ChassisCommand last_cmd;
  if (std::abs(cmd.linear_vel) < 1e-6 && std::abs(cmd.angular_vel) < 1e-6 &&
      std::abs(last_cmd.linear_vel) < 1e-6 && std::abs(last_cmd.angular_vel) < 1e-6)
    return;
  last_cmd = cmd;

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
  filtered_linear_vel_ = 0.0;
  filtered_angular_vel_ = 0.0;
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
  stat.add("odom_source", odom_source_);
  stat.add("msg_count", msg_count_.load());
  stat.add("error_count", error_count_.load());

  // base_vel 数据超时检测
  if (odom_source_ == "base_vel" && base_vel_initialized_)
  {
    double age = (ros::Time::now() - last_base_vel_time_).toSec();
    stat.add("base_vel_age", age);
    if (age > 1.0)
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
                        "base_vel data timeout");
  }

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

  // 型号/版本信息
  if (msg.find("model:") == 0 || msg.find("hfls_version:") == 0)
  {
    ROS_INFO_STREAM("[LepuDriver] " << msg);
    return;
  }

  // 定位未完成
  if (msg.find("nav:pose:notfound") == 0)
  {
    ROS_WARN_THROTTLE(5.0, "[LepuDriver] Localizing... (nav:pose:notfound)");
    return;
  }

  ros::Time now = ros::Time::now();

  // 里程计核心算法 (integrateMotion/encoderDelta) 保留在类中供未来硬件扩展

  // 解析 base_vel — 速度积分里程计（官方 §9）
  if (odom_source_ == "base_vel")
  {
    double bv, bw;
    if (parseBaseVel(msg, bv, bw))
    {
      boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
      bv *= odom_linear_scale_;
      bw *= odom_angular_scale_;
      linear_vel_ = bv;
      angular_vel_ = bw;

      if (!base_vel_initialized_)
      {
        last_base_vel_time_ = now;
        base_vel_initialized_ = true;
      }
      else
      {
        double dt = (now - last_base_vel_time_).toSec();
        last_base_vel_time_ = now;
        if (dt > 0.0 && dt <= dt_max_)
        {
          integrateMotion(odom_x_, odom_y_, odom_yaw_, linear_vel_, angular_vel_,
                         bv * dt, bw * dt, dt);
        }
      }
    }
  }
  // 解析 nav_pose — 绝对位姿里程计（官方 §27/§37）
  else if (odom_source_ == "nav_pose")
  {
    double nx, ny, nyaw;
    double data_ts = -1.0;   // nav:time_pose 带时间戳(≥0), nav:pose 不带(保持-1)
    if (parseNavPose(msg, nx, ny, nyaw, &data_ts))
    {
      bool has_data_ts = (data_ts >= 0.0);

      boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
      if (!pose_origin_set_)
      {
        pose_origin_x_ = nx;
        pose_origin_y_ = ny;
        pose_origin_yaw_ = nyaw;
        pose_origin_set_ = true;
        odom_x_ = 0.0; odom_y_ = 0.0; odom_yaw_ = 0.0;
        last_nav_x_ = 0.0; last_nav_y_ = 0.0; last_nav_yaw_ = 0.0;
        filtered_linear_vel_ = 0.0; filtered_angular_vel_ = 0.0;
        last_nav_pose_time_ = now;
        if (has_data_ts) last_data_ts_ = data_ts;
        return;
      }

      // 坐标转换：SLAM绝对坐标 → 相对origin的odom坐标
      double rel_x = nx - pose_origin_x_;
      double rel_y = ny - pose_origin_y_;
      double cos_yaw = std::cos(pose_origin_yaw_);
      double sin_yaw = std::sin(pose_origin_yaw_);
      odom_x_ = rel_x * cos_yaw + rel_y * sin_yaw;
      odom_y_ = -rel_x * sin_yaw + rel_y * cos_yaw;
      odom_yaw_ = ChassisDriver::normalizeAngle(nyaw - pose_origin_yaw_);

      // 速度计算：仅用带时间戳的 nav:time_pose 计算速度
      // SLAM 每周期同时发送 nav:time_pose(有ts) + nav:pose(无ts)，
      // 若两条都计算速度，第二条 dx≈0 会通过低通滤波把速度拉向零。
      // 策略：nav:time_pose → 计算速度并更新 last_nav；nav:pose → 只更新位置。
      const double MIN_VEL_DT = 0.02;  // 最小有效计算间隔 (50Hz)
      if (has_data_ts && last_data_ts_ > 0.0 && last_nav_pose_time_.isValid())
      {
        double nav_dt = data_ts - last_data_ts_;
        if (nav_dt >= MIN_VEL_DT && nav_dt <= dt_max_)
        {
          // 原始速度（位置差分）
          double dx = odom_x_ - last_nav_x_;
          double dy = odom_y_ - last_nav_y_;
          double raw_speed = std::hypot(dx, dy) / nav_dt;
          // 位移投影到车头方向确定正负：向前为正，后退为负
          double mid_yaw = (odom_yaw_ + last_nav_yaw_) * 0.5;
          double forward_dot = dx * std::cos(mid_yaw) + dy * std::sin(mid_yaw);
          double raw_linear = (forward_dot >= 0.0 ? 1.0 : -1.0) * raw_speed;
          double raw_angular = ChassisDriver::normalizeAngle(odom_yaw_ - last_nav_yaw_) / nav_dt;

          // 一阶低通滤波: v_f = α·v_raw + (1-α)·v_f_prev
          double alpha = vel_lpf_alpha_;
          double flv = alpha * raw_linear + (1.0 - alpha) * filtered_linear_vel_;
          double fav = alpha * raw_angular + (1.0 - alpha) * filtered_angular_vel_;

          // 变化率限幅 — 防止SLAM重定位等导致的瞬时跳变
          double max_dv = max_vel_change_ * nav_dt;
          if (std::abs(flv - filtered_linear_vel_) > max_dv)
            flv = filtered_linear_vel_ + std::copysign(max_dv, flv - filtered_linear_vel_);
          if (std::abs(fav - filtered_angular_vel_) > max_dv)
            fav = filtered_angular_vel_ + std::copysign(max_dv, fav - filtered_angular_vel_);

          filtered_linear_vel_ = flv;
          filtered_angular_vel_ = fav;
          linear_vel_ = flv;
          angular_vel_ = fav;

          ROS_INFO_THROTTLE(2.0,
            "nav_vel: odom(%.3f,%.3f) last(%.3f,%.3f) dxy=(%.4f,%.4f) dt=%.4f raw=(%.3f,%.3f) filt=(%.3f,%.3f)",
            odom_x_, odom_y_, last_nav_x_, last_nav_y_,
            odom_x_ - last_nav_x_, odom_y_ - last_nav_y_,
            nav_dt, raw_linear, raw_angular, flv, fav);

          // 有效计算后更新 last_nav 和 last_data_ts
          last_nav_x_ = odom_x_; last_nav_y_ = odom_y_; last_nav_yaw_ = odom_yaw_;
          last_nav_pose_time_ = now;
          last_data_ts_ = data_ts;
        }
        // dt 异常时跳过，不更新 last_nav，位移累积到下一有效帧
      }
    }
  }
}

double LepuDriver::encoderDelta(int32_t current, int32_t previous) const
{
  // 保留供未来硬件扩展使用。当前编码器值始终为 0
  return static_cast<double>(current - previous);
}


}  // namespace chassis_interface

// pluginlib 注册
PLUGINLIB_EXPORT_CLASS(chassis_interface::LepuDriver, chassis_interface::ChassisDriver)
