#include <chassis_interface/chassis_bridge.h>
#include <boost/algorithm/clamp.hpp>

namespace chassis_interface
{

ChassisBridge::ChassisBridge()
{
}

ChassisBridge::~ChassisBridge()
{
  // 必须先销毁 driver（它可能持有对 ClassLoader 中类型的引用）
  // 然后才能销毁 driver_loader_
  if (driver_)
  {
    driver_->disconnect();
    driver_.reset();
  }
  driver_loader_.reset();
}

bool ChassisBridge::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  nh_ = nh;
  pnh_ = pnh;

  // ---- 读取参数 ----
  pnh_.param<std::string>("driver_plugin", driver_plugin_,
                          "chassis_interface::LepuDriver");
  pnh_.param<std::string>("odom_topic", odom_topic_, "odom");
  pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
  pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param<int>("odom_queue_size", odom_queue_size_, 50);
  pnh_.param<int>("cmd_vel_queue_size", cmd_vel_queue_size_, 1);
  pnh_.param<double>("max_linear_speed", max_linear_speed_, 0.3);
  pnh_.param<double>("max_angular_speed", max_angular_speed_, 0.8);
  pnh_.param<double>("cmd_vel_timeout", cmd_vel_timeout_, 0.2);
  pnh_.param<double>("publish_rate", publish_rate_, 50.0);
  pnh_.param<double>("diag_rate", diag_rate_, 1.0);

  // 参数校验
  if (publish_rate_ <= 0.0) {
    ROS_ERROR("[ChassisBridge] publish_rate must be > 0, got %.1f", publish_rate_);
    return false;
  }

  // 协方差参数（对角线元素，其余为0）
  pnh_.param<double>("cov_pose_xx", cov_pose_xx_, 0.05);
  pnh_.param<double>("cov_pose_yy", cov_pose_yy_, 0.05);
  pnh_.param<double>("cov_pose_yawyaw", cov_pose_yawyaw_, 0.1);
  pnh_.param<double>("cov_twist_xx", cov_twist_xx_, 0.05);
  pnh_.param<double>("cov_twist_yawyaw", cov_twist_yawyaw_, 0.05);

  // ---- ROS 接口初始化 ----
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, odom_queue_size_);
  cmd_vel_feedback_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_feedback", 10);
  cmd_vel_feedback_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("cmd_vel_feedback_odom", 10);
  odom_yaw_pub_ = nh_.advertise<std_msgs::Float32>("odom_yaw", 10);
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      cmd_vel_topic_, cmd_vel_queue_size_, &ChassisBridge::cmdVelCallback, this);

  enable_srv_ = pnh_.advertiseService("enable", &ChassisBridge::onEnable, this);
  disable_srv_ = pnh_.advertiseService("disable", &ChassisBridge::onDisable, this);
  reset_odom_srv_ = pnh_.advertiseService("reset_odom", &ChassisBridge::onResetOdom, this);
  emergency_srv_ = pnh_.advertiseService("emergency_stop", &ChassisBridge::onEmergencyStop, this);

  // ---- 诊断 ----
  diag_updater_.reset(new diagnostic_updater::Updater(nh_));
  diag_updater_->setHardwareID("chassis_bridge");
  diag_updater_->add("Chassis Driver", this, &ChassisBridge::produceDiagnostics);

  // ---- 加载驱动 ----
  transitionTo(LifecycleState::CONFIGURING);
  if (!loadDriver())
  {
    ROS_ERROR("[ChassisBridge] Failed to load driver plugin: %s", driver_plugin_.c_str());
    return false;
  }

  // ---- 启动定时器 ----
  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                   &ChassisBridge::publishLoop, this);
  diag_timer_ = nh_.createTimer(ros::Duration(1.0 / diag_rate_),
                                &ChassisBridge::diagTimerCallback, this);
  transitionTo(LifecycleState::READY);
  // 默认自动使能
  enabled_ = true;
  transitionTo(LifecycleState::RUNNING);

  ROS_INFO("[ChassisBridge] Initialized. Driver: %s, publish_rate: %.1f Hz",
           driver_plugin_.c_str(), publish_rate_);
  ROS_INFO("[ChassisBridge] Services: enable/disable/reset_odom/emergency_stop");
  return true;
}

// ============================================================
// 服务回调
// ============================================================
bool ChassisBridge::onEnable(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res)
{
  if (!driver_)
  {
    res.success = false;
    res.message = "Cannot enable — no driver loaded";
    return true;
  }
  if (lifecycle_ == LifecycleState::EMERGENCY)
  {
    res.success = false;
    res.message = "Cannot enable — chassis is in EMERGENCY state. Reset first.";
    return true;
  }
  enabled_ = true;
  transitionTo(LifecycleState::RUNNING);
  res.success = true;
  res.message = "Chassis enabled";
  ROS_INFO("[ChassisBridge] Enabled");
  return true;
}

bool ChassisBridge::onDisable(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res)
{
  enabled_ = false;
  if (driver_)
  {
    driver_->writeCommand(ChassisCommand());  // 发零速度
  }
  transitionTo(LifecycleState::READY);
  res.success = true;
  res.message = "Chassis disabled";
  ROS_INFO("[ChassisBridge] Disabled");
  return true;
}

bool ChassisBridge::onResetOdom(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res)
{
  if (driver_)
  {
    driver_->resetOdom();
    res.success = true;
    res.message = "Odometry reset to zero";
    ROS_INFO("[ChassisBridge] Odometry reset");
  }
  else
  {
    res.success = false;
    res.message = "No driver loaded";
  }
  return true;
}

bool ChassisBridge::onEmergencyStop(std_srvs::Trigger::Request& req,
                                    std_srvs::Trigger::Response& res)
{
  enabled_ = false;
  if (driver_)
  {
    driver_->emergencyStop();
  }
  transitionTo(LifecycleState::EMERGENCY);
  res.success = true;
  res.message = "EMERGENCY STOP activated! Manual reset required.";
  ROS_WARN("[ChassisBridge] EMERGENCY STOP!");
  return true;
}

// ============================================================
// cmd_vel 回调
// ============================================================
void ChassisBridge::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!enabled_ || lifecycle_ != LifecycleState::RUNNING) return;

  std::lock_guard<std::mutex> lock(cmd_mutex_);
  current_cmd_.linear_vel = boost::algorithm::clamp(msg->linear.x,
                                                     -max_linear_speed_, max_linear_speed_);
  current_cmd_.angular_vel = boost::algorithm::clamp(msg->angular.z,
                                                      -max_angular_speed_, max_angular_speed_);
  current_cmd_.stamp = ros::Time::now();
  last_cmd_time_ = current_cmd_.stamp;
}

// ============================================================
// 定时器回调
// ============================================================
void ChassisBridge::publishLoop(const ros::TimerEvent& event)
{
  if (!driver_) return;

  // 超时处理 — cmd_vel 超过 timeout 未更新则自动停止
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (enabled_ && (ros::Time::now() - last_cmd_time_).toSec() > cmd_vel_timeout_)
    {
      current_cmd_.linear_vel = 0.0;
      current_cmd_.angular_vel = 0.0;
    }
  }

  // 下发指令
  if (enabled_ && lifecycle_ == LifecycleState::RUNNING)
  {
    ChassisCommand cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd = current_cmd_;
    }
    driver_->writeCommand(cmd);
  }
  else
  {
    driver_->writeCommand(ChassisCommand());  // 零速
  }

  // 读取状态
  ChassisState state = driver_->readState();

  if (!state.is_connected && lifecycle_ == LifecycleState::RUNNING)
  {
    transitionTo(LifecycleState::ERROR);
  }
  else if (state.is_connected && lifecycle_ == LifecycleState::ERROR)
  {
    transitionTo(LifecycleState::RUNNING);
  }

  // 发布里程计
  nav_msgs::Odometry odom;
  odom.header.stamp = state.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw);
  odom.twist.twist.linear.x = state.linear_vel;
  odom.twist.twist.angular.z = state.angular_vel;
  // 协方差（通过参数可调，默认值适用于一般编码器里程计）
  odom.pose.covariance[0]  = cov_pose_xx_;      //  X   pos_x
  odom.pose.covariance[7]  = cov_pose_yy_;      //  Y   pos_y
  odom.pose.covariance[35] = cov_pose_yawyaw_;   // Yaw  pos_yaw
  odom.twist.covariance[0]  = cov_twist_xx_;     //  X   vel_x
  odom.twist.covariance[35] = cov_twist_yawyaw_; // Yaw  vel_yaw
  odom_pub_.publish(odom);

  // 速度反馈 — MPC 闭环控制使用
  geometry_msgs::Twist feedback;
  feedback.linear.x  = state.linear_vel;
  feedback.angular.z = state.angular_vel;
  cmd_vel_feedback_pub_.publish(feedback);

  // /cmd_vel_feedback_odom — Odometry格式的速度反馈
  cmd_vel_feedback_odom_pub_.publish(odom);

  // /odom_yaw — 单独的偏航角
  std_msgs::Float32 yaw_msg;
  yaw_msg.data = state.yaw;
  odom_yaw_pub_.publish(yaw_msg);

  // TF odom→base_link 由 xrobot_driver_odom_fusion 和 xrobot_ukf_localization
  // 基于 /odom 数据融合发布，chassis_bridge_node 不再重复发布以避免干扰 TF 树
}

void ChassisBridge::diagTimerCallback(const ros::TimerEvent& event)
{
  if (diag_updater_)
  {
    diag_updater_->update();
  }
}

void ChassisBridge::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!driver_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No driver loaded");
    return;
  }
  driver_->getDiagnostics(stat);
}

// ============================================================
// 状态机
// ============================================================
void ChassisBridge::transitionTo(LifecycleState new_state)
{
  LifecycleState old = lifecycle_.exchange(new_state);
  const char* names[] = {"UNCONFIGURED", "CONFIGURING", "READY", "RUNNING", "ERROR", "EMERGENCY"};
  ROS_INFO("[ChassisBridge] State: %s -> %s",
           names[static_cast<int>(old)], names[static_cast<int>(new_state)]);
}

bool ChassisBridge::loadDriver()
{
  try
  {
    driver_loader_.reset(new pluginlib::ClassLoader<ChassisDriver>(
        "chassis_interface", "chassis_interface::ChassisDriver"));
    driver_.reset(driver_loader_->createUnmanagedInstance(driver_plugin_));
    if (!driver_->connect(pnh_))
    {
      ROS_ERROR("[ChassisBridge] Driver connect() failed");
      return false;
    }
    return true;
  }
  catch (const pluginlib::PluginlibException& e)
  {
    ROS_ERROR("[ChassisBridge] Pluginlib error: %s", e.what());
    return false;
  }
}

}  // namespace chassis_interface
