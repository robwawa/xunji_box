#ifndef CHASSIS_INTERFACE_CHASSIS_BRIDGE_H
#define CHASSIS_INTERFACE_CHASSIS_BRIDGE_H

#include <chassis_interface/chassis_driver.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_srvs/Trigger.h>
#include <pluginlib/class_loader.h>
#include <memory>
#include <atomic>
#include <mutex>

namespace chassis_interface
{

// ============================================================
// ChassisBridge — 通用底盘桥接节点
//
// 职责：
//  1. 通过 pluginlib 加载具体驱动插件
//  2. 管理生命周期状态机
//  3. 统一发布 /odom + TF + /diagnostics
//  4. 统一订阅 /cmd_vel 并限幅后下发
//  5. 提供 enable/disable/reset_odom/emergency_stop 服务
// ============================================================
class ChassisBridge
{
public:
  ChassisBridge();
  ~ChassisBridge();

  bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  // ---- 服务回调 ----
  bool onEnable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool onDisable(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool onResetOdom(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool onEmergencyStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // ---- 订阅回调 ----
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  // ---- 定时器回调 ----
  void publishLoop(const ros::TimerEvent& event);
  void diagTimerCallback(const ros::TimerEvent& event);

  // ---- 诊断回调 ----
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // ---- 状态机 ----
  void transitionTo(LifecycleState new_state);
  bool loadDriver();

  // ---- ROS 接口 ----
  ros::NodeHandle nh_, pnh_;

  // 参数
  std::string driver_plugin_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  int odom_queue_size_;
  int cmd_vel_queue_size_;
  double max_linear_speed_;
  double max_angular_speed_;
  double cmd_vel_timeout_;
  double publish_rate_;
  double diag_rate_;

  // 里程计协方差参数
  double cov_pose_xx_, cov_pose_yy_, cov_pose_yawyaw_;
  double cov_twist_xx_, cov_twist_yawyaw_;

  // 话题
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_vel_sub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // 服务
  ros::ServiceServer enable_srv_;
  ros::ServiceServer disable_srv_;
  ros::ServiceServer reset_odom_srv_;
  ros::ServiceServer emergency_srv_;

  // 定时器
  ros::Timer publish_timer_;
  ros::Timer diag_timer_;

  // 诊断
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;

  // 驱动
  std::unique_ptr<pluginlib::ClassLoader<ChassisDriver>> driver_loader_;
  std::unique_ptr<ChassisDriver> driver_;

  // 状态
  std::atomic<LifecycleState> lifecycle_{LifecycleState::UNCONFIGURED};
  std::mutex cmd_mutex_;
  ChassisCommand current_cmd_;
  ros::Time last_cmd_time_;
  std::atomic<bool> enabled_{false};
};

}  // namespace chassis_interface

#endif  // CHASSIS_INTERFACE_CHASSIS_BRIDGE_H
