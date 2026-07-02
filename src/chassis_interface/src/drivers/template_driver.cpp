#include <chassis_interface/chassis_driver.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread/shared_mutex.hpp>
#include <cmath>
#include <mutex>

namespace chassis_interface
{

/**
 * TemplateDriver — 新底盘驱动开发模板 (差分驱动 / 串口通信)
 *
 * 接入步骤:
 *   1. cp template_driver.cpp your_driver.cpp
 *   2. 全局替换 TemplateDriver → YourDriver, "template" → "your_chassis"
 *   3. 实现所有标记 TODO 的代码段
 *   4. 在 chassis_interface_plugin.xml 中注册
 *   5. 在 CMakeLists.txt add_library 中添加 src/drivers/your_driver.cpp
 *   6. 复制 config/template_chassis.yaml → your_chassis.yaml, 修改参数
 *   7. catkin_make && roslaunch
 *
 * 底盘通信方式参考:
 *   - 串口:  参考 LepuDriver / PosixSerial
 *   - CAN:   使用 SocketCAN (socket, bind, read/write)
 *   - 以太网: 使用 ROS topic 或 TCP socket
 *   - 这里以串口为例
 */
class TemplateDriver : public ChassisDriver
{
public:
  TemplateDriver();
  ~TemplateDriver() override;

  // ---- ChassisDriver 接口 ----
  bool connect(ros::NodeHandle& nh) override;
  void disconnect() override;
  ChassisState readState() override;
  void writeCommand(const ChassisCommand& cmd) override;
  void emergencyStop() override;
  void resetOdom() override;
  void getDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) override;
  std::string getDriverName() const override { return "template"; }

private:
  // ---- 硬件通信 (TODO: 替换为你的通信方式) ----
  bool openHardware();
  void closeHardware();
  bool isHardwareOpen() const;
  int writeHardware(const uint8_t* data, size_t len);
  int readHardware(uint8_t* buf, size_t max_len, int timeout_ms);

  // ---- 串口示例 ----
  std::string port_;
  int baudrate_;
  int hw_fd_;  // 文件描述符 (串口/socket/CAN)

  // ---- 协议转换 (TODO: 实现你的底盘协议) ----
  void sendVelocity(double linear, double angular);
  void sendStop();

  // 里程计算法 inherit from ChassisDriver::integrateMotion / normalizeAngle

  mutable boost::shared_mutex state_mutex_;
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_yaw_ = 0.0;
  double linear_vel_ = 0.0, angular_vel_ = 0.0;

  // ---- 底盘参数 ----
  double wheel_separation_;     // 两轮间距 (m)
  double wheel_radius_;         // 轮子半径 (m)
  int encoder_ticks_per_rev_;   // 编码器每圈脉冲
  double meters_per_tick_;
  double odom_linear_scale_ = 1.0;
  double odom_angular_scale_ = 1.0;

  // ---- 诊断 ----
  std::atomic<int> msg_count_{0};
  std::atomic<int> err_count_{0};
  ros::Time last_data_time_;
};

// ============================================================
// 构造/析构
// ============================================================
TemplateDriver::TemplateDriver()
  : hw_fd_(-1)
{
}

TemplateDriver::~TemplateDriver()
{
  disconnect();
}

// ============================================================
// 连接底盘
// ============================================================
bool TemplateDriver::connect(ros::NodeHandle& nh)
{
  // 读取参数 (这些参数在 your_chassis.yaml 中配置)
  nh.param<std::string>("port", port_, "/dev/ttyUSB0");
  nh.param<int>("baudrate", baudrate_, 115200);
  nh.param<double>("wheel_separation", wheel_separation_, 0.30);
  nh.param<double>("wheel_radius", wheel_radius_, 0.10);
  nh.param<int>("encoder_ticks_per_rev", encoder_ticks_per_rev_, 4096);
  nh.param<double>("odom_linear_scale", odom_linear_scale_, 1.0);
  nh.param<double>("odom_angular_scale", odom_angular_scale_, 1.0);

  meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / std::max(encoder_ticks_per_rev_, 1);

  // 打开硬件
  if (!openHardware())
  {
    ROS_ERROR("[TemplateDriver] Failed to open %s", port_.c_str());
    return false;
  }

  ROS_INFO("[TemplateDriver] Connected %s @ %d, sep=%.3f rad=%.3f",
           port_.c_str(), baudrate_, wheel_separation_, wheel_radius_);

  // TODO: 发送初始化指令 (示例)
  // writeHardware(...);

  last_data_time_ = ros::Time::now();
  return true;
}

void TemplateDriver::disconnect()
{
  sendStop();
  closeHardware();
}

// ============================================================
// 读取状态 (必须实现 — 从底盘获取编码器/位姿/速度)
// ============================================================
ChassisState TemplateDriver::readState()
{
  ChassisState state;

  // TODO: 从底盘读取编码器或速度数据
  // 示例 — 假设通过串口读取到左右轮编码器值:
  // int32_t left_enc = 0, right_enc = 0;
  // if (readEncodersFromHardware(left_enc, right_enc)) { ... }
  //
  // 编码器 → 里程计 的差速模型:
  //   delta_left  = (left_enc  - last_left_enc_)  * meters_per_tick_;
  //   delta_right = (right_enc - last_right_enc_) * meters_per_tick_;
  //   delta_center = 0.5 * (delta_left + delta_right);
  //   delta_yaw    = (delta_right - delta_left) / wheel_separation_;
  //   integrateMotion(delta_center, delta_yaw, dt);

  {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    state.x = odom_x_;
    state.y = odom_y_;
    state.yaw = odom_yaw_;
    state.linear_vel = linear_vel_;
    state.angular_vel = angular_vel_;
  }

  state.is_connected = isHardwareOpen();
  state.stamp = ros::Time::now();

  if (!state.is_connected)
  {
    state.error_code = 1;
    state.status_msg = "Hardware disconnected";
  }

  return state;
}

// ============================================================
// 下发指令 (必须实现 — 将速度转为底盘协议)
// ============================================================
void TemplateDriver::writeCommand(const ChassisCommand& cmd)
{
  if (!isHardwareOpen()) return;

  // TODO: 将 cmd.linear_vel / cmd.angular_vel 转为你的底盘协议
  // 示例 — 差速底盘转换为左右轮速度:
  //   double left_vel  = cmd.linear_vel - cmd.angular_vel * wheel_separation_ / 2.0;
  //   double right_vel = cmd.linear_vel + cmd.angular_vel * wheel_separation_ / 2.0;
  //   sendVelocity(left_vel, right_vel);

  sendVelocity(cmd.linear_vel, cmd.angular_vel);
}

// ============================================================
// 紧急停止
// ============================================================
void TemplateDriver::emergencyStop()
{
  sendStop();

  boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
  linear_vel_ = 0.0;
  angular_vel_ = 0.0;
}

// ============================================================
// 重置里程计
// ============================================================
void TemplateDriver::resetOdom()
{
  boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  linear_vel_ = 0.0;
  angular_vel_ = 0.0;
}

// ============================================================
// 诊断
// ============================================================
void TemplateDriver::getDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!isHardwareOpen())
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Hardware disconnected");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
  stat.add("port", port_);
  stat.add("msg_count", msg_count_.load());
  stat.add("err_count", err_count_.load());
}

// ============================================================
// 里程计算法 (通用 — 可直接复用)
// ============================================================
// integrateMotion/normalizeAngle 已在 ChassisDriver 基类中实现，直接调用

// ============================================================
// 硬件层 — 串口示例 (TODO: 替换为你的通信方式)
// ============================================================
bool TemplateDriver::openHardware()
{
  // 示例：POSIX 串口
  // hw_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  // ... 配置 termios ...
  // return hw_fd_ >= 0;

  hw_fd_ = 0;  // 桩：表示已连接
  return true;
}

void TemplateDriver::closeHardware()
{
  if (hw_fd_ >= 0) { /* ::close(hw_fd_); */ hw_fd_ = -1; }
}

bool TemplateDriver::isHardwareOpen() const
{
  return hw_fd_ >= 0;
}

int TemplateDriver::writeHardware(const uint8_t* data, size_t len)
{
  if (hw_fd_ < 0) return -1;
  // return ::write(hw_fd_, data, len);
  return static_cast<int>(len);  // 桩
}

int TemplateDriver::readHardware(uint8_t* buf, size_t max_len, int timeout_ms)
{
  if (hw_fd_ < 0) return -1;
  // select + ::read ...
  return 0;  // 桩
}

// ============================================================
// 协议层 — 速度控制 (TODO: 实现你的底盘协议帧)
// ============================================================
void TemplateDriver::sendVelocity(double linear, double angular)
{
  // TODO: 构造并发送速度指令帧
  // 示例 — 乐普协议: app_vel[linear,angular]
  // 示例 — 某底盘自定义帧: <header> <len> <left_rpm> <right_rpm> <crc>
}

void TemplateDriver::sendStop()
{
  // TODO: 构造并发送停止帧
  // 大多数底盘: 发送零速度指令即可
  sendVelocity(0.0, 0.0);
}

}  // namespace chassis_interface

PLUGINLIB_EXPORT_CLASS(chassis_interface::TemplateDriver, chassis_interface::ChassisDriver)
