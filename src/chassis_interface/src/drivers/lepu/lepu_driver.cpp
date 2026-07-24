#include <chassis_interface/chassis_driver.h>
#include <chassis_interface/drivers/lepu/lepu_protocol.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread/shared_mutex.hpp>
#include <cmath>
#include <thread>
#include <atomic>

namespace chassis_interface
{

enum class ModeState
{
  DISCONNECTED,
  SETTING_MODE,
  WAITING_CONFIRMATION,
  WAITING_ODOM,
  READY
};

enum class HandshakeStep
{
  ZERO_SPEED,
  HEARTBEAT,
  SET_MODE,
  QUERY_MODE
};

constexpr double HANDSHAKE_COMMAND_GAP = 0.5;
constexpr double HANDSHAKE_RESPONSE_WAIT = 1.0;
constexpr double HANDSHAKE_HEARTBEAT_INTERVAL = 5.0;

static const char* modeStateName(ModeState state)
{
  switch (state)
  {
    case ModeState::DISCONNECTED:          return "DISCONNECTED";
    case ModeState::SETTING_MODE:          return "SETTING_MODE";
    case ModeState::WAITING_CONFIRMATION:  return "WAITING_CONFIRMATION";
    case ModeState::WAITING_ODOM:          return "WAITING_ODOM";
    case ModeState::READY:                 return "READY";
  }
  return "UNKNOWN";
}

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
  void handleConnectionChanged(bool connected);
  void sessionTimerCallback(const ros::WallTimerEvent&);
  void heartbeatTimerCallback(const ros::TimerEvent&);
  void markValidOdomLocked(const ros::Time& stamp);
  void resetOdomSessionLocked(bool reset_position);
  double encoderDelta(int32_t current, int32_t previous) const;

  // ---- 参数 ----
  std::string port_;
  int baudrate_;
  bool enable_pose_report_;
  std::string odom_source_;      // "base_vel" (默认) 或 "nav_pose"
  std::string nav_mode_;         // "mapping"(默认) "navi" "remap"
  double odom_linear_scale_;
  double odom_angular_scale_;
  double reconnect_interval_;
  double data_timeout_;

  // ---- 串口链路 ----
  mutable std::mutex serial_mutex_;
  std::unique_ptr<LepuSerialLink> serial_;
  std::atomic<bool> link_connected_{false};
  std::atomic<bool> session_ready_{false};
  std::atomic<bool> valid_odom_received_{false};
  std::atomic<bool> timeout_reconnect_requested_{false};

  // ---- 状态（保护锁） ----
  mutable boost::shared_mutex state_mutex_;
  double odom_x_, odom_y_, odom_yaw_;
  double linear_vel_, angular_vel_;
  // 编码器值：官方协议无编码器上报，固定为 0
  int32_t last_left_encoder_ = 0, last_right_encoder_ = 0;
  ros::Time last_base_vel_time_;
  bool base_vel_initialized_ = false;
  double dt_max_ = 0.5;            // 最大有效时间间隔 (s)

  // nav_pose 会话锚点：每次重连后以断联前 odom 位姿重新锚定
  bool pose_origin_set_ = false;
  double pose_origin_x_ = 0.0, pose_origin_y_ = 0.0, pose_origin_yaw_ = 0.0;
  double pose_anchor_odom_x_ = 0.0, pose_anchor_odom_y_ = 0.0;
  double pose_anchor_odom_yaw_ = 0.0;

  // ---- 定时器 ----
  ros::WallTimer session_timer_;
  ros::Timer heartbeat_timer_;

  // nav_pose 速度计算
  double last_nav_x_ = 0.0, last_nav_y_ = 0.0, last_nav_yaw_ = 0.0;
  ros::Time last_nav_pose_time_;
  double last_data_ts_ = 0.0;        // 上一帧 nav:time_pose 自带时间戳
  bool last_data_ts_valid_ = false;   // last_data_ts_ 是否已初始化（替代 >0.0 检查）
  ros::Time last_valid_state_stamp_;
  ros::WallTime last_valid_data_wall_time_;
  bool last_valid_data_time_set_ = false;

  // 低通滤波 — 抑制位置差分噪声
  double vel_lpf_alpha_ = 0.3;       // 滤波系数 (0~1, 越小越平滑)
  double filtered_linear_vel_ = 0.0;
  double filtered_angular_vel_ = 0.0;
  double max_vel_change_ = 0.5;      // m/s², 单周期最大线速度变化
  double max_angular_vel_change_ = 1.0; // rad/s², 单周期最大角速度变化

  // ---- 诊断 ----
  std::atomic<int> msg_count_{0};
  std::atomic<int> error_count_{0};

  // ---- 指令去重（重连时重置，确保恢复后首条指令为零速）----
  std::mutex command_mutex_;
  ChassisCommand last_command_;
  bool last_command_valid_ = false;

  // ---- 模式切换确认 ----
  std::atomic<bool> nav_mode_confirmed_{false};
  std::atomic<int> mode_retry_count_{0};
  std::atomic<int> reported_nav_mode_{-1};
  std::atomic<ModeState> mode_state_{ModeState::DISCONNECTED};
  std::atomic<HandshakeStep> handshake_step_{HandshakeStep::ZERO_SPEED};
  std::atomic<double> handshake_action_after_{0.0};
  std::atomic<double> next_handshake_heartbeat_{0.0};
  std::atomic<double> odom_accept_after_{0.0};
  std::atomic<bool> ever_confirmed_session_{false};
  std::string nav_mode_expected_response_;  // e.g. "model:2" for mapping
  std::string nav_mode_command_;
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
  nh.param<double>("max_angular_vel_change", max_angular_vel_change_, 1.0);
  nh.param<double>("reconnect_interval", reconnect_interval_, 1.0);
  nh.param<double>("data_timeout", data_timeout_, 1.0);

  reconnect_interval_ = std::max(0.05, reconnect_interval_);
  data_timeout_ = std::max(0.1, data_timeout_);

  if (!getNavModeProtocol(nav_mode_, nav_mode_command_,
                          nav_mode_expected_response_))
  {
    ROS_ERROR("[LepuDriver] Invalid nav_mode '%s'; expected navi, mapping, or remap",
              nav_mode_.c_str());
    return false;
  }

  // 创建串口链路。即使设备当前不存在，后台线程也会持续等待稳定别名出现。
  serial_.reset(new LepuSerialLink(port_, baudrate_,
    std::bind(&LepuDriver::handleMessage, this, std::placeholders::_1),
    std::bind(&LepuDriver::handleConnectionChanged, this, std::placeholders::_1),
    reconnect_interval_));

  if (!serial_->open())
  {
    ROS_ERROR("[LepuDriver] Failed to start serial link manager");
    return false;
  }

  ROS_INFO("[LepuDriver] Serial manager started for %s @ %d baud; nav_mode=%s",
           port_.c_str(), baudrate_, nav_mode_.c_str());

  // 严格启动逻辑：收到配置模式的精确回复前不完成驱动初始化。
  // 乐普固件对连续命令较敏感，因此命令之间留出处理时间，并按文档先
  // 发送模式切换、等待自动上报，未确认时再查询当前模式。
  ros::WallRate serial_wait_rate(20.0);
  while (ros::ok())
  {
    bool serial_open = false;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_open = serial_ && serial_->isOpen();
    }
    if (serial_open) break;
    serial_wait_rate.sleep();
  }
  if (!ros::ok()) return false;

  auto send_startup_command = [this](const std::string& command) -> bool
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_ || !serial_->isOpen()) return false;
    return serial_->sendCommand(command);
  };

  if (!send_startup_command("app_vel[0,0]")) return false;
  ros::WallDuration(0.25).sleep();
  if (!send_startup_command("keep_connect")) return false;
  ros::WallDuration(HANDSHAKE_COMMAND_GAP).sleep();

  bool send_mode_next = true;
  double next_heartbeat =
      ros::WallTime::now().toSec() + HANDSHAKE_HEARTBEAT_INTERVAL;
  while (ros::ok())
  {
    // 要求模式确认连续稳定一个命令间隔，避免旧 model:1 帧晚到后误放行。
    if (nav_mode_confirmed_)
    {
      ros::WallDuration(HANDSHAKE_COMMAND_GAP).sleep();
      if (nav_mode_confirmed_) break;
      continue;
    }

    const double now = ros::WallTime::now().toSec();
    double wait_after_send = HANDSHAKE_RESPONSE_WAIT;
    if (now >= next_heartbeat)
    {
      if (!send_startup_command("keep_connect"))
      {
        ROS_ERROR("[LepuDriver] Serial lost while sending startup heartbeat");
        return false;
      }
      next_heartbeat = now + HANDSHAKE_HEARTBEAT_INTERVAL;
      wait_after_send = HANDSHAKE_COMMAND_GAP;
    }
    else if (send_mode_next)
    {
      mode_state_ = ModeState::SETTING_MODE;
      mode_retry_count_++;
      if (!send_startup_command(nav_mode_command_))
      {
        ROS_ERROR("[LepuDriver] Serial lost while setting startup mode");
        return false;
      }
      send_mode_next = false;
    }
    else
    {
      mode_state_ = ModeState::WAITING_CONFIRMATION;
      if (!send_startup_command("model:request"))
      {
        ROS_ERROR("[LepuDriver] Serial lost while querying startup mode");
        return false;
      }
      send_mode_next = true;
    }
    ros::WallDuration(wait_after_send).sleep();
  }
  if (!ros::ok()) return false;

  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_ || !serial_->isOpen()) return false;
    if (enable_pose_report_)
      serial_->sendCommand("nav:get_pose[open?on]");
  }

  // 重连后的会话恢复使用相同的分步握手。
  session_timer_ = nh.createWallTimer(
      ros::WallDuration(0.1), &LepuDriver::sessionTimerCallback, this);
  // 心跳保活 — 官方协议要求 5s 一次 (§1 心跳包)
  heartbeat_timer_ = nh.createTimer(
      ros::Duration(5.0), &LepuDriver::heartbeatTimerCallback, this);

  return true;
}

void LepuDriver::disconnect()
{
  session_timer_.stop();
  heartbeat_timer_.stop();
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
  double data_age = -1.0;
  {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    state.x = odom_x_;
    state.y = odom_y_;
    state.yaw = odom_yaw_;
    state.linear_vel = linear_vel_;
    state.angular_vel = angular_vel_;
    state.left_encoder = last_left_encoder_;
    state.right_encoder = last_right_encoder_;
    state.stamp = last_valid_state_stamp_;
    if (last_valid_data_time_set_)
      data_age = (ros::WallTime::now() - last_valid_data_wall_time_).toSec();
  }

  bool link_open = false;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    link_open = serial_ && serial_->isOpen();
  }

  bool data_fresh = data_age >= 0.0 && data_age <= data_timeout_;
  state.is_connected = link_open && session_ready_ && data_fresh;

  if (link_open && session_ready_ && !data_fresh &&
      !timeout_reconnect_requested_.exchange(true))
  {
    error_count_++;
    session_ready_ = false;
    ROS_ERROR("[LepuDriver] Odometry data timeout (age=%.3fs), reconnecting",
              data_age);
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_) serial_->requestReconnect();
  }

  if (!state.is_connected)
  {
    state.linear_vel = 0.0;
    state.angular_vel = 0.0;
    state.error_code = 1;
    if (!link_open)
      state.status_msg = "Serial disconnected";
    else if (!nav_mode_confirmed_)
      state.status_msg = "Reconnecting: waiting for nav mode confirmation";
    else if (!valid_odom_received_)
      state.status_msg = "Reconnecting: waiting for odometry";
    else
      state.status_msg = "Odometry data timeout";
  }

  return state;
}

void LepuDriver::writeCommand(const ChassisCommand& cmd)
{
  if (!session_ready_) return;

  bool is_zero = std::abs(cmd.linear_vel) < 1e-6 &&
                 std::abs(cmd.angular_vel) < 1e-6;
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    bool last_is_zero = last_command_valid_ &&
                        std::abs(last_command_.linear_vel) < 1e-6 &&
                        std::abs(last_command_.angular_vel) < 1e-6;
    if (is_zero && last_is_zero) return;
  }

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_ || !serial_->isOpen())
  {
    return;
  }

  char buf[64];
  snprintf(buf, sizeof(buf), "app_vel[%.3f,%.3f]", cmd.linear_vel, cmd.angular_vel);
  if (serial_->sendCommand(buf))
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    last_command_ = cmd;
    last_command_valid_ = true;
  }
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
  base_vel_initialized_ = false;
  last_data_ts_valid_ = false;
}

void LepuDriver::resetOdomSessionLocked(bool reset_position)
{
  if (reset_position)
  {
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_yaw_ = 0.0;
  }
  linear_vel_ = 0.0;
  angular_vel_ = 0.0;
  filtered_linear_vel_ = 0.0;
  filtered_angular_vel_ = 0.0;
  base_vel_initialized_ = false;
  pose_origin_set_ = false;
  last_data_ts_valid_ = false;
  last_valid_data_time_set_ = false;
}

void LepuDriver::getDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  bool link_open = false;
  int reconnect_count = 0;
  int read_errors = 0;
  int write_errors = 0;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_)
    {
      link_open = serial_->isOpen();
      reconnect_count = serial_->reconnectCount();
      read_errors = serial_->readErrorCount();
      write_errors = serial_->writeErrorCount();
    }
  }

  double data_age = -1.0;
  {
    boost::shared_lock<boost::shared_mutex> lock(state_mutex_);
    if (last_valid_data_time_set_)
      data_age = (ros::WallTime::now() - last_valid_data_wall_time_).toSec();
  }

  if (!link_open)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Serial disconnected");
  }
  else if (!nav_mode_confirmed_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                 "Requested chassis mode not confirmed");
  }
  else if (!session_ready_ || data_age < 0.0 || data_age > data_timeout_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                 "Serial reconnecting or odometry data stale");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }

  stat.add("port", port_);
  stat.add("odom_source", odom_source_);
  stat.add("connection_state",
           !link_open ? "DISCONNECTED" :
           (session_ready_ ? "READY" : "RECONNECTING"));
  stat.add("mode_state", modeStateName(mode_state_.load()));
  stat.add("expected_nav_mode", nav_mode_expected_response_);
  stat.add("last_odom_age", data_age);
  stat.add("nav_mode_confirmed", nav_mode_confirmed_.load());
  stat.add("reported_nav_mode", reported_nav_mode_.load());
  stat.add("mode_retry_count", mode_retry_count_.load());
  stat.add("reconnect_count", reconnect_count);
  stat.add("serial_read_errors", read_errors);
  stat.add("serial_write_errors", write_errors);
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
void LepuDriver::handleConnectionChanged(bool connected)
{
  link_connected_ = connected;
  session_ready_ = false;
  valid_odom_received_ = false;
  timeout_reconnect_requested_ = false;
  nav_mode_confirmed_ = false;
  mode_retry_count_ = 0;
  reported_nav_mode_ = -1;
  mode_state_ = connected ? ModeState::SETTING_MODE : ModeState::DISCONNECTED;
  handshake_step_ = HandshakeStep::ZERO_SPEED;
  const double now = ros::WallTime::now().toSec();
  handshake_action_after_ = now;
  next_handshake_heartbeat_ = now + 0.25;
  odom_accept_after_ = 0.0;

  {
    boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
    resetOdomSessionLocked(false);
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_command_valid_ = false;
  }

  if (connected)
  {
    ROS_INFO("[LepuDriver] Serial connected; initializing chassis session");
  }
  else
  {
    error_count_++;
    ROS_ERROR("[LepuDriver] Serial disconnected; odometry output suspended");
  }
}

void LepuDriver::sessionTimerCallback(const ros::WallTimerEvent&)
{
  if (!link_connected_ || session_ready_) return;

  const double now = ros::WallTime::now().toSec();
  if (now < handshake_action_after_.load()) return;

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_ || !serial_->isOpen()) return;

  if (nav_mode_confirmed_)
  {
    mode_state_ = ModeState::WAITING_ODOM;
    if (enable_pose_report_ && !valid_odom_received_)
      serial_->sendCommand("nav:get_pose[open?on]");
    handshake_action_after_ = now + HANDSHAKE_RESPONSE_WAIT;
    return;
  }

  HandshakeStep step = handshake_step_.load();
  if (step != HandshakeStep::ZERO_SPEED &&
      step != HandshakeStep::HEARTBEAT &&
      now >= next_handshake_heartbeat_.load())
  {
    if (serial_->sendCommand("keep_connect"))
    {
      next_handshake_heartbeat_ = now + HANDSHAKE_HEARTBEAT_INTERVAL;
      handshake_action_after_ = now + HANDSHAKE_COMMAND_GAP;
    }
    return;
  }

  switch (step)
  {
    case HandshakeStep::ZERO_SPEED:
      if (serial_->sendCommand("app_vel[0,0]"))
      {
        handshake_step_ = HandshakeStep::HEARTBEAT;
        handshake_action_after_ = now + 0.25;
      }
      break;

    case HandshakeStep::HEARTBEAT:
      if (serial_->sendCommand("keep_connect"))
      {
        next_handshake_heartbeat_ = now + HANDSHAKE_HEARTBEAT_INTERVAL;
        handshake_step_ = HandshakeStep::SET_MODE;
        handshake_action_after_ = now + HANDSHAKE_COMMAND_GAP;
      }
      break;

    case HandshakeStep::SET_MODE:
      mode_state_ = ModeState::SETTING_MODE;
      mode_retry_count_++;
      if (serial_->sendCommand(nav_mode_command_))
      {
        handshake_step_ = HandshakeStep::QUERY_MODE;
        handshake_action_after_ = now + HANDSHAKE_RESPONSE_WAIT;
      }
      break;

    case HandshakeStep::QUERY_MODE:
      mode_state_ = ModeState::WAITING_CONFIRMATION;
      if (serial_->sendCommand("model:request"))
      {
        handshake_step_ = HandshakeStep::SET_MODE;
        handshake_action_after_ = now + HANDSHAKE_RESPONSE_WAIT;
      }
      break;
  }
}

void LepuDriver::heartbeatTimerCallback(const ros::TimerEvent&)
{
  // 未完成会话时由分步握手负责心跳，避免与模式命令紧邻发送。
  if (!link_connected_ || !session_ready_) return;
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_ && serial_->isOpen()) serial_->sendCommand("keep_connect");
}

void LepuDriver::markValidOdomLocked(const ros::Time& stamp)
{
  last_valid_state_stamp_ = stamp;
  last_valid_data_wall_time_ = ros::WallTime::now();
  last_valid_data_time_set_ = true;
  valid_odom_received_ = true;
  timeout_reconnect_requested_ = false;

  if (nav_mode_confirmed_ && !session_ready_.exchange(true))
  {
    mode_state_ = ModeState::READY;
    ROS_INFO("[LepuDriver] Confirmed mode and valid odometry; output resumed");
  }
}

void LepuDriver::handleMessage(const std::string& msg)
{
  msg_count_++;

  // 型号/版本信息
  if (msg.find("model:") == 0)
  {
    ROS_INFO_STREAM("[LepuDriver] " << msg);
    int reported_mode = parseNavModeResponse(msg);
    if (reported_mode < 0) return;

    reported_nav_mode_ = reported_mode;
    if (msg == nav_mode_expected_response_)
    {
      if (!nav_mode_confirmed_.exchange(true))
      {
        const bool reset_position = !ever_confirmed_session_.exchange(true);
        {
          boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
          resetOdomSessionLocked(reset_position);
        }
        valid_odom_received_ = false;
        session_ready_ = false;
        mode_state_ = ModeState::WAITING_ODOM;
        const double now = ros::WallTime::now().toSec();
        // 丢弃模式切换前已进入串口/解析队列的旧坐标帧。
        odom_accept_after_ = now + 0.5;
        ROS_INFO("[LepuDriver] Requested nav mode confirmed: %s",
                 nav_mode_expected_response_.c_str());
      }
    }
    else
    {
      bool was_operational = session_ready_.exchange(false) ||
                             nav_mode_confirmed_.exchange(false);
      valid_odom_received_ = false;
      {
        boost::unique_lock<boost::shared_mutex> lock(state_mutex_);
        resetOdomSessionLocked(false);
      }
      mode_state_ = ModeState::WAITING_CONFIRMATION;
      if (was_operational) error_count_++;
      ROS_ERROR_THROTTLE(2.0,
          "[LepuDriver] Wrong chassis mode model:%d; expected %s. "
          "Odometry suspended.",
          reported_mode, nav_mode_expected_response_.c_str());
    }
    return;
  }
  if (msg.find("hfls_version:") == 0)
  {
    ROS_INFO_STREAM("[LepuDriver] " << msg);
    return;
  }

  // 指定模式未得到精确确认前，丢弃所有里程计数据，避免错误模式污染状态。
  if (!nav_mode_confirmed_) return;
  // 模式确认后的短暂稳定窗口用于排空旧模式的残留位姿帧。
  if (ros::WallTime::now().toSec() < odom_accept_after_.load()) return;

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
      markValidOdomLocked(now);
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
        // 保存断联前的 odom 位姿，新会话第一帧只建立坐标锚点，不制造跳变。
        pose_anchor_odom_x_ = odom_x_;
        pose_anchor_odom_y_ = odom_y_;
        pose_anchor_odom_yaw_ = odom_yaw_;
        pose_origin_x_ = nx;
        pose_origin_y_ = ny;
        pose_origin_yaw_ = nyaw;
        pose_origin_set_ = true;
        last_nav_x_ = odom_x_;
        last_nav_y_ = odom_y_;
        last_nav_yaw_ = odom_yaw_;
        filtered_linear_vel_ = 0.0; filtered_angular_vel_ = 0.0;
        linear_vel_ = 0.0; angular_vel_ = 0.0;
        last_nav_pose_time_ = now;
        last_data_ts_valid_ = has_data_ts;
        if (has_data_ts)
        {
          last_data_ts_ = data_ts;
        }
        markValidOdomLocked(now);
        return;
      }

      // 坐标转换：当前 SLAM 会话坐标 → 以断联前位姿为锚点的连续 odom 坐标
      double rel_x = nx - pose_origin_x_;
      double rel_y = ny - pose_origin_y_;
      double cos_source = std::cos(pose_origin_yaw_);
      double sin_source = std::sin(pose_origin_yaw_);
      double local_x = rel_x * cos_source + rel_y * sin_source;
      double local_y = -rel_x * sin_source + rel_y * cos_source;
      double cos_anchor = std::cos(pose_anchor_odom_yaw_);
      double sin_anchor = std::sin(pose_anchor_odom_yaw_);
      odom_x_ = pose_anchor_odom_x_ +
                local_x * cos_anchor - local_y * sin_anchor;
      odom_y_ = pose_anchor_odom_y_ +
                local_x * sin_anchor + local_y * cos_anchor;
      odom_yaw_ = ChassisDriver::normalizeAngle(
          pose_anchor_odom_yaw_ +
          ChassisDriver::normalizeAngle(nyaw - pose_origin_yaw_));

      // 速度计算：仅用带时间戳的 nav:time_pose 计算速度
      // SLAM 每周期同时发送 nav:time_pose(有ts) + nav:pose(无ts)，
      // 若两条都计算速度，第二条 dx≈0 会通过低通滤波把速度拉向零。
      // 策略：nav:time_pose → 计算速度并更新 last_nav；nav:pose → 只更新位置。
      const double MIN_VEL_DT = 0.02;  // 最小有效计算间隔 (50Hz)
      if (has_data_ts && last_data_ts_valid_ && last_nav_pose_time_.isValid())
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
          double max_dw = max_angular_vel_change_ * nav_dt;
          if (std::abs(flv - filtered_linear_vel_) > max_dv)
            flv = filtered_linear_vel_ + std::copysign(max_dv, flv - filtered_linear_vel_);
          if (std::abs(fav - filtered_angular_vel_) > max_dw)
            fav = filtered_angular_vel_ + std::copysign(max_dw, fav - filtered_angular_vel_);

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
        else if (nav_dt > dt_max_ || nav_dt < 0.0)
        {
          // dt 超限/回退：不计算速度，但更新基准防止后续帧异常
          // 同时更新 last_nav 防止下一帧 dx 跨越异常区间导致速度尖峰
          last_data_ts_ = data_ts;
          last_nav_x_ = odom_x_; last_nav_y_ = odom_y_; last_nav_yaw_ = odom_yaw_;
          last_nav_pose_time_ = now;
        }
        // dt < MIN_VEL_DT: 跳过，不更新 last_nav 也不更新时间戳，位移累积
      }
      markValidOdomLocked(now);
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
