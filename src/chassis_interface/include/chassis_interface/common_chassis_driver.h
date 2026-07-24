#ifndef CHASSIS_INTERFACE_COMMON_CHASSIS_DRIVER_H
#define CHASSIS_INTERFACE_COMMON_CHASSIS_DRIVER_H

#include <chassis_interface/chassis_driver.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace chassis_interface
{

/**
 * 一帧与具体通信协议无关的硬件反馈。
 *
 * 不同父类需要填写的字段：
 *   WheelOdometryDriver: left_encoder、right_encoder
 *   VelocityOdometryDriver: linear_vel、angular_vel
 *   DirectOdometryDriver: x、y、yaw，可选速度字段
 *   六类运动学父类: wheel_speeds、steering_angles
 */
struct HardwareFeedback
{
  ros::Time stamp;
  bool is_connected = true;
  int error_code = 0;
  std::string status_msg;

  int64_t left_encoder = 0;
  int64_t right_encoder = 0;

  double linear_vel = 0.0;
  double angular_vel = 0.0;
  bool velocity_valid = false;

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  // 轮地接触方向的有符号线速度，单位 m/s。
  std::vector<double> wheel_speeds;
  // 车体坐标系中的实际滚动方向，单位 rad，逆时针为正。
  std::vector<double> steering_angles;
};

enum class OdometryModel
{
  WHEEL_ENCODER,
  VELOCITY_INTEGRATION,
  DIRECT_POSE,
  DIFFERENTIAL,
  DOUBLE_DIFFERENTIAL,
  SINGLE_STEER,
  DOUBLE_STEER,
  FOUR_STEER,
  GENERAL_KINEMATICS
};

class CommonChassisDriver : public ChassisDriver
{
public:
  explicit CommonChassisDriver(OdometryModel model);
  ~CommonChassisDriver() override = default;

  bool connect(ros::NodeHandle& node) final;
  void disconnect() final;
  ChassisState readState() final;
  void writeCommand(const ChassisCommand& cmd) final;
  void emergencyStop() final;
  void resetOdom() final;
  void getDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) final;

protected:
  // 普通驱动仅需实现这两个协议扩展点。
  virtual bool sendCommandToHardware(const ChassisCommand& cmd) = 0;
  virtual bool receiveStateFromHardware(HardwareFeedback& feedback) = 0;

  virtual bool onConfigure(ros::NodeHandle& node);
  virtual bool onConnect();
  virtual void onDisconnect();
  virtual bool isHardwareConnected() const;
  virtual bool sendEmergencyStopToHardware(const ChassisCommand& stop);
  virtual void onResetOdometry();
  virtual void appendDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat);

  ros::NodeHandle& node() { return node_; }
  const ros::NodeHandle& node() const { return node_; }

private:
  void processFeedback(const HardwareFeedback& feedback);
  void processWheelFeedback(const HardwareFeedback& feedback, double dt);
  void processVelocityFeedback(const HardwareFeedback& feedback, double dt);
  void processDirectFeedback(const HardwareFeedback& feedback, double dt);
  bool processKinematicFeedback(const HardwareFeedback& feedback, double dt);
  bool computeKinematicTwist(
    const HardwareFeedback& feedback,
    double& vx, double& vy, double& wz) const;
  bool solveWheelConstraints(
    const std::vector<double>& speeds,
    const std::vector<double>& angles,
    const std::vector<double>& positions_x,
    const std::vector<double>& positions_y,
    bool constrain_lateral,
    double& vx, double& vy, double& wz) const;
  double encoderDelta(int64_t current, int64_t previous) const;
  void clearOdometryLocked();

  const OdometryModel odometry_model_;
  ros::NodeHandle node_;

  mutable std::mutex state_mutex_;
  ChassisState state_;
  std::atomic<bool> connected_{false};
  bool feedback_initialized_ = false;
  ros::Time last_feedback_stamp_;
  ros::Time last_feedback_receive_time_;

  double wheel_separation_ = 0.30;
  double wheel_radius_ = 0.10;
  int encoder_ticks_per_rev_ = 4096;
  int encoder_bits_ = 32;
  double meters_per_tick_ = 0.0;
  int64_t last_left_encoder_ = 0;
  int64_t last_right_encoder_ = 0;

  double odom_linear_scale_ = 1.0;
  double odom_angular_scale_ = 1.0;
  double max_feedback_dt_ = 0.5;
  double feedback_timeout_ = 1.0;

  double wheelbase_ = 1.0;
  double wheel_distance_ = 0.0;
  double steer_offset_y_ = 0.0;
  double kinematic_damping_ = 1e-9;
  int num_wheels_ = 0;
  std::vector<double> wheel_positions_x_;
  std::vector<double> wheel_positions_y_;

  bool direct_origin_pending_ = false;
  bool direct_origin_valid_ = false;
  double direct_origin_x_ = 0.0;
  double direct_origin_y_ = 0.0;
  double direct_origin_yaw_ = 0.0;
  double last_direct_x_ = 0.0;
  double last_direct_y_ = 0.0;
  double last_direct_yaw_ = 0.0;

  std::atomic<uint64_t> feedback_count_{0};
  std::atomic<uint64_t> command_count_{0};
  std::atomic<uint64_t> error_count_{0};
};

class WheelOdometryDriver : public CommonChassisDriver
{
public:
  WheelOdometryDriver()
    : CommonChassisDriver(OdometryModel::WHEEL_ENCODER) {}
};

class VelocityOdometryDriver : public CommonChassisDriver
{
public:
  VelocityOdometryDriver()
    : CommonChassisDriver(OdometryModel::VELOCITY_INTEGRATION) {}
};

class DirectOdometryDriver : public CommonChassisDriver
{
public:
  DirectOdometryDriver()
    : CommonChassisDriver(OdometryModel::DIRECT_POSE) {}
};

class DifferentialOdometryDriver : public CommonChassisDriver
{
public:
  DifferentialOdometryDriver()
    : CommonChassisDriver(OdometryModel::DIFFERENTIAL) {}
};

class DoubleDifferentialOdometryDriver : public CommonChassisDriver
{
public:
  DoubleDifferentialOdometryDriver()
    : CommonChassisDriver(OdometryModel::DOUBLE_DIFFERENTIAL) {}
};

class SingleSteerOdometryDriver : public CommonChassisDriver
{
public:
  SingleSteerOdometryDriver()
    : CommonChassisDriver(OdometryModel::SINGLE_STEER) {}
};

class DoubleSteerOdometryDriver : public CommonChassisDriver
{
public:
  DoubleSteerOdometryDriver()
    : CommonChassisDriver(OdometryModel::DOUBLE_STEER) {}
};

class FourSteerOdometryDriver : public CommonChassisDriver
{
public:
  FourSteerOdometryDriver()
    : CommonChassisDriver(OdometryModel::FOUR_STEER) {}
};

class GeneralKinematicsOdometryDriver : public CommonChassisDriver
{
public:
  GeneralKinematicsOdometryDriver()
    : CommonChassisDriver(OdometryModel::GENERAL_KINEMATICS) {}
};

}  // namespace chassis_interface

#endif  // CHASSIS_INTERFACE_COMMON_CHASSIS_DRIVER_H
