#include <chassis_interface/common_chassis_driver.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace chassis_interface
{
namespace
{

bool isKinematicModel(OdometryModel model)
{
  return model == OdometryModel::DIFFERENTIAL ||
         model == OdometryModel::DOUBLE_DIFFERENTIAL ||
         model == OdometryModel::SINGLE_STEER ||
         model == OdometryModel::DOUBLE_STEER ||
         model == OdometryModel::FOUR_STEER ||
         model == OdometryModel::GENERAL_KINEMATICS;
}

bool allFinite(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(),
    [](double value) { return std::isfinite(value); });
}

bool solveSymmetric3x3(
  std::array<std::array<double, 3>, 3> matrix,
  std::array<double, 3> rhs,
  std::array<double, 3>& solution)
{
  for (size_t column = 0; column < 3; ++column) {
    size_t pivot = column;
    for (size_t row = column + 1; row < 3; ++row) {
      if (std::abs(matrix[row][column]) >
          std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) < 1e-15) {
      return false;
    }
    if (pivot != column) {
      std::swap(matrix[pivot], matrix[column]);
      std::swap(rhs[pivot], rhs[column]);
    }
    for (size_t row = column + 1; row < 3; ++row) {
      const double factor = matrix[row][column] / matrix[column][column];
      for (size_t col = column; col < 3; ++col) {
        matrix[row][col] -= factor * matrix[column][col];
      }
      rhs[row] -= factor * rhs[column];
    }
  }

  for (int row = 2; row >= 0; --row) {
    double value = rhs[static_cast<size_t>(row)];
    for (size_t col = static_cast<size_t>(row) + 1; col < 3; ++col) {
      value -= matrix[static_cast<size_t>(row)][col] * solution[col];
    }
    solution[static_cast<size_t>(row)] =
      value / matrix[static_cast<size_t>(row)][static_cast<size_t>(row)];
  }
  return allFinite({solution[0], solution[1], solution[2]});
}

}  // namespace

CommonChassisDriver::CommonChassisDriver(OdometryModel model)
  : odometry_model_(model)
{
}

bool CommonChassisDriver::connect(ros::NodeHandle& node)
{
  node_ = node;

  node_.param<double>("odom_linear_scale", odom_linear_scale_, 1.0);
  node_.param<double>("odom_angular_scale", odom_angular_scale_, 1.0);
  node_.param<double>("max_feedback_dt", max_feedback_dt_, 0.5);
  node_.param<double>("feedback_timeout", feedback_timeout_, 1.0);

  if (odometry_model_ == OdometryModel::WHEEL_ENCODER ||
      odometry_model_ == OdometryModel::DIFFERENTIAL) {
    node_.param<double>("wheel_separation", wheel_separation_, 0.30);
  }

  if (odometry_model_ == OdometryModel::WHEEL_ENCODER) {
    node_.param<double>("wheel_radius", wheel_radius_, 0.10);
    node_.param<int>("encoder_ticks_per_rev", encoder_ticks_per_rev_, 4096);
    node_.param<int>("encoder_bits", encoder_bits_, 32);

    if (!std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0 ||
        !std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0 ||
        encoder_ticks_per_rev_ <= 0 || encoder_bits_ < 0 || encoder_bits_ > 62) {
      ROS_ERROR("[CommonChassisDriver] invalid wheel odometry parameters");
      return false;
    }
    meters_per_tick_ =
      2.0 * M_PI * wheel_radius_ / static_cast<double>(encoder_ticks_per_rev_);
  }

  if (isKinematicModel(odometry_model_)) {
    node_.param<double>("kinematic_damping", kinematic_damping_, 1e-9);

    if (odometry_model_ == OdometryModel::DOUBLE_DIFFERENTIAL ||
        odometry_model_ == OdometryModel::SINGLE_STEER ||
        odometry_model_ == OdometryModel::DOUBLE_STEER) {
      node_.param<double>("wheelbase", wheelbase_, 1.0);
    }
    if (odometry_model_ == OdometryModel::SINGLE_STEER) {
      node_.param<double>("steer_offset_y", steer_offset_y_, 0.0);
    }
    if (odometry_model_ == OdometryModel::DOUBLE_STEER) {
      node_.param<double>("wheel_distance", wheel_distance_, 0.0);
    }
    if (odometry_model_ == OdometryModel::FOUR_STEER) {
      num_wheels_ = 4;
      node_.param<std::vector<double>>(
        "wheel_positions_x", wheel_positions_x_,
        std::vector<double>{0.5, 0.5, -0.5, -0.5});
      node_.param<std::vector<double>>(
        "wheel_positions_y", wheel_positions_y_,
        std::vector<double>{0.3, -0.3, 0.3, -0.3});
    } else if (odometry_model_ == OdometryModel::GENERAL_KINEMATICS) {
      node_.param<int>("num_wheels", num_wheels_, 4);
      node_.param<std::vector<double>>(
        "wheel_positions_x", wheel_positions_x_,
        std::vector<double>{0.5, 0.5, -0.5, -0.5});
      node_.param<std::vector<double>>(
        "wheel_positions_y", wheel_positions_y_,
        std::vector<double>{0.3, -0.3, 0.3, -0.3});
    }

    const bool invalid_separation =
      odometry_model_ == OdometryModel::DIFFERENTIAL &&
      (!std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0);
    const bool needs_wheelbase =
      odometry_model_ == OdometryModel::DOUBLE_DIFFERENTIAL ||
      odometry_model_ == OdometryModel::SINGLE_STEER ||
      odometry_model_ == OdometryModel::DOUBLE_STEER;
    const bool invalid_wheelbase =
      needs_wheelbase && (!std::isfinite(wheelbase_) || wheelbase_ <= 0.0);
    const bool invalid_wheel_distance =
      odometry_model_ == OdometryModel::DOUBLE_STEER &&
      (!std::isfinite(wheel_distance_) || wheel_distance_ < 0.0);
    const bool invalid_positions =
      (odometry_model_ == OdometryModel::FOUR_STEER ||
       odometry_model_ == OdometryModel::GENERAL_KINEMATICS) &&
      (num_wheels_ < 1 || num_wheels_ > 4 ||
       wheel_positions_x_.size() != static_cast<size_t>(num_wheels_) ||
       wheel_positions_y_.size() != static_cast<size_t>(num_wheels_) ||
       !allFinite(wheel_positions_x_) || !allFinite(wheel_positions_y_));

    if (!std::isfinite(kinematic_damping_) || kinematic_damping_ <= 0.0 ||
        !std::isfinite(steer_offset_y_) || invalid_separation ||
        invalid_wheelbase || invalid_wheel_distance || invalid_positions) {
      ROS_ERROR("[CommonChassisDriver] invalid kinematic odometry parameters");
      return false;
    }
  }

  if (!std::isfinite(odom_linear_scale_) ||
      !std::isfinite(odom_angular_scale_) ||
      !std::isfinite(max_feedback_dt_) ||
      !std::isfinite(feedback_timeout_) ||
      max_feedback_dt_ <= 0.0 || feedback_timeout_ < 0.0) {
    ROS_ERROR(
      "[CommonChassisDriver] max_feedback_dt must be > 0 and feedback_timeout >= 0");
    return false;
  }

  if (!onConfigure(node_) || !onConnect()) {
    connected_ = false;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    clearOdometryLocked();
    connected_ = true;
    state_.is_connected = true;
    state_.stamp = ros::Time::now();
    last_feedback_stamp_ = state_.stamp;
    last_feedback_receive_time_ = state_.stamp;
  }
  return true;
}

void CommonChassisDriver::disconnect()
{
  if (connected_ && isHardwareConnected()) {
    ChassisCommand stop;
    stop.stamp = ros::Time::now();
    if (!sendCommandToHardware(stop)) {
      ++error_count_;
    }
  }
  onDisconnect();

  std::lock_guard<std::mutex> lock(state_mutex_);
  connected_ = false;
  state_.is_connected = false;
  state_.linear_vel = 0.0;
  state_.linear_vel_y = 0.0;
  state_.angular_vel = 0.0;
}

ChassisState CommonChassisDriver::readState()
{
  HardwareFeedback feedback;
  const bool has_feedback = receiveStateFromHardware(feedback);
  if (has_feedback) {
    if (feedback.stamp.isZero()) {
      feedback.stamp = ros::Time::now();
    }
    processFeedback(feedback);
    ++feedback_count_;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool transport_connected = isHardwareConnected();
  state_.is_connected = connected_ && transport_connected;

  if (feedback_timeout_ > 0.0 && connected_) {
    const double age =
      (ros::Time::now() - last_feedback_receive_time_).toSec();
    if (age > feedback_timeout_) {
      state_.linear_vel = 0.0;
      state_.linear_vel_y = 0.0;
      state_.angular_vel = 0.0;
      state_.is_connected = false;
      state_.error_code = 2;
      state_.status_msg = "Feedback timeout";
    }
  }

  if (!state_.is_connected) {
    state_.linear_vel = 0.0;
    state_.linear_vel_y = 0.0;
    state_.angular_vel = 0.0;
    if (state_.error_code == 0) {
      state_.error_code = 1;
      state_.status_msg = "Hardware disconnected";
    }
  }
  return state_;
}

void CommonChassisDriver::writeCommand(const ChassisCommand& cmd)
{
  if (!isHardwareConnected() || !sendCommandToHardware(cmd)) {
    ++error_count_;
    return;
  }
  ++command_count_;
}

void CommonChassisDriver::emergencyStop()
{
  ChassisCommand stop;
  stop.stamp = ros::Time::now();
  if (isHardwareConnected() && !sendEmergencyStopToHardware(stop)) {
    ++error_count_;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.linear_vel = 0.0;
  state_.linear_vel_y = 0.0;
  state_.angular_vel = 0.0;
}

void CommonChassisDriver::resetOdom()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    clearOdometryLocked();
    if (odometry_model_ == OdometryModel::DIRECT_POSE) {
      direct_origin_pending_ = true;
    }
  }
  onResetOdometry();
}

void CommonChassisDriver::getDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  ChassisState snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot = state_;
  }

  if (!snapshot.is_connected) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      snapshot.status_msg.empty() ? "Hardware disconnected" : snapshot.status_msg);
  } else if (snapshot.error_code != 0) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
      snapshot.status_msg.empty() ? "Chassis warning" : snapshot.status_msg);
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
  stat.add("driver", getDriverName());
  stat.add("feedback_count", feedback_count_.load());
  stat.add("command_count", command_count_.load());
  stat.add("error_count", error_count_.load());
  stat.add("odom_x", snapshot.x);
  stat.add("odom_y", snapshot.y);
  stat.add("odom_yaw", snapshot.yaw);
  stat.add("linear_vel", snapshot.linear_vel);
  stat.add("linear_vel_y", snapshot.linear_vel_y);
  stat.add("angular_vel", snapshot.angular_vel);
  appendDiagnostics(stat);
}

bool CommonChassisDriver::onConfigure(ros::NodeHandle&)
{
  return true;
}

bool CommonChassisDriver::onConnect()
{
  return true;
}

void CommonChassisDriver::onDisconnect()
{
}

bool CommonChassisDriver::isHardwareConnected() const
{
  return connected_;
}

bool CommonChassisDriver::sendEmergencyStopToHardware(
  const ChassisCommand& stop)
{
  return sendCommandToHardware(stop);
}

void CommonChassisDriver::onResetOdometry()
{
}

void CommonChassisDriver::appendDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper&)
{
}

void CommonChassisDriver::processFeedback(const HardwareFeedback& feedback)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  connected_ = feedback.is_connected;
  state_.is_connected = feedback.is_connected;
  state_.error_code = feedback.error_code;
  state_.status_msg = feedback.status_msg;
  state_.stamp = feedback.stamp;
  if (!feedback.is_connected) {
    ++error_count_;
    return;
  }

  const bool invalid_velocity =
    !std::isfinite(feedback.linear_vel) || !std::isfinite(feedback.angular_vel);
  const bool invalid_pose =
    !std::isfinite(feedback.x) || !std::isfinite(feedback.y) ||
    !std::isfinite(feedback.yaw);
  if ((odometry_model_ == OdometryModel::VELOCITY_INTEGRATION &&
       invalid_velocity) ||
      (odometry_model_ == OdometryModel::DIRECT_POSE &&
       (invalid_pose || (feedback.velocity_valid && invalid_velocity)))) {
    state_.error_code = 3;
    state_.status_msg = "Invalid non-finite feedback";
    ++error_count_;
    return;
  }

  double dt = 0.0;
  if (feedback_initialized_) {
    dt = (feedback.stamp - last_feedback_stamp_).toSec();
  }

  if (odometry_model_ == OdometryModel::WHEEL_ENCODER) {
    processWheelFeedback(feedback, dt);
  } else if (odometry_model_ == OdometryModel::VELOCITY_INTEGRATION) {
    processVelocityFeedback(feedback, dt);
  } else if (odometry_model_ == OdometryModel::DIRECT_POSE) {
    processDirectFeedback(feedback, dt);
  } else if (!processKinematicFeedback(feedback, dt)) {
    state_.linear_vel = 0.0;
    state_.linear_vel_y = 0.0;
    state_.angular_vel = 0.0;
    state_.error_code = 4;
    state_.status_msg = "Invalid kinematic feedback";
    ++error_count_;
  }

  last_feedback_stamp_ = feedback.stamp;
  last_feedback_receive_time_ = ros::Time::now();
  feedback_initialized_ = true;
}

void CommonChassisDriver::processWheelFeedback(
  const HardwareFeedback& feedback, double dt)
{
  state_.left_encoder = static_cast<int32_t>(feedback.left_encoder);
  state_.right_encoder = static_cast<int32_t>(feedback.right_encoder);

  if (!feedback_initialized_ || dt <= 0.0 || dt > max_feedback_dt_) {
    last_left_encoder_ = feedback.left_encoder;
    last_right_encoder_ = feedback.right_encoder;
    state_.linear_vel = 0.0;
    state_.linear_vel_y = 0.0;
    state_.angular_vel = 0.0;
    return;
  }

  const double delta_left =
    encoderDelta(feedback.left_encoder, last_left_encoder_) *
    meters_per_tick_ * odom_linear_scale_;
  const double delta_right =
    encoderDelta(feedback.right_encoder, last_right_encoder_) *
    meters_per_tick_ * odom_linear_scale_;
  last_left_encoder_ = feedback.left_encoder;
  last_right_encoder_ = feedback.right_encoder;

  const double delta_center = 0.5 * (delta_left + delta_right);
  const double delta_yaw =
    (delta_right - delta_left) / wheel_separation_ * odom_angular_scale_;
  integrateMotion(state_.x, state_.y, state_.yaw,
    state_.linear_vel, state_.angular_vel, delta_center, delta_yaw, dt);
}

void CommonChassisDriver::processVelocityFeedback(
  const HardwareFeedback& feedback, double dt)
{
  state_.linear_vel = feedback.linear_vel * odom_linear_scale_;
  state_.linear_vel_y = 0.0;
  state_.angular_vel = feedback.angular_vel * odom_angular_scale_;
  if (!feedback_initialized_ || dt <= 0.0 || dt > max_feedback_dt_) {
    return;
  }

  integrateMotion(state_.x, state_.y, state_.yaw,
    state_.linear_vel, state_.angular_vel,
    state_.linear_vel * dt, state_.angular_vel * dt, dt);
}

void CommonChassisDriver::processDirectFeedback(
  const HardwareFeedback& feedback, double dt)
{
  if (direct_origin_pending_) {
    direct_origin_x_ = feedback.x;
    direct_origin_y_ = feedback.y;
    direct_origin_yaw_ = feedback.yaw;
    direct_origin_valid_ = true;
    direct_origin_pending_ = false;
  }

  double x = feedback.x;
  double y = feedback.y;
  double yaw = normalizeAngle(feedback.yaw);
  if (direct_origin_valid_) {
    const double dx = feedback.x - direct_origin_x_;
    const double dy = feedback.y - direct_origin_y_;
    const double c = std::cos(direct_origin_yaw_);
    const double s = std::sin(direct_origin_yaw_);
    x = dx * c + dy * s;
    y = -dx * s + dy * c;
    yaw = normalizeAngle(feedback.yaw - direct_origin_yaw_);
  }

  if (feedback.velocity_valid) {
    state_.linear_vel = feedback.linear_vel * odom_linear_scale_;
    state_.linear_vel_y = 0.0;
    state_.angular_vel = feedback.angular_vel * odom_angular_scale_;
  } else if (feedback_initialized_ && dt > 0.0 && dt <= max_feedback_dt_) {
    const double dx = x - last_direct_x_;
    const double dy = y - last_direct_y_;
    const double mid_yaw = last_direct_yaw_ +
      0.5 * normalizeAngle(yaw - last_direct_yaw_);
    state_.linear_vel =
      (dx * std::cos(mid_yaw) + dy * std::sin(mid_yaw)) / dt *
      odom_linear_scale_;
    state_.linear_vel_y =
      (-dx * std::sin(mid_yaw) + dy * std::cos(mid_yaw)) / dt *
      odom_linear_scale_;
    state_.angular_vel =
      normalizeAngle(yaw - last_direct_yaw_) / dt * odom_angular_scale_;
  } else {
    state_.linear_vel = 0.0;
    state_.linear_vel_y = 0.0;
    state_.angular_vel = 0.0;
  }

  state_.x = x;
  state_.y = y;
  state_.yaw = yaw;
  last_direct_x_ = x;
  last_direct_y_ = y;
  last_direct_yaw_ = yaw;
}

bool CommonChassisDriver::processKinematicFeedback(
  const HardwareFeedback& feedback, double dt)
{
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  if (!computeKinematicTwist(feedback, vx, vy, wz)) {
    return false;
  }

  if (!feedback_initialized_ || dt <= 0.0 || dt > max_feedback_dt_) {
    state_.linear_vel = 0.0;
    state_.linear_vel_y = 0.0;
    state_.angular_vel = 0.0;
    return true;
  }

  wz *= odom_angular_scale_;
  const double delta_yaw = wz * dt;
  const double mid_yaw = state_.yaw + 0.5 * delta_yaw;
  state_.x += (vx * std::cos(mid_yaw) - vy * std::sin(mid_yaw)) * dt;
  state_.y += (vx * std::sin(mid_yaw) + vy * std::cos(mid_yaw)) * dt;
  state_.yaw = normalizeAngle(state_.yaw + delta_yaw);
  state_.linear_vel = vx;
  state_.linear_vel_y = vy;
  state_.angular_vel = wz;
  return true;
}

bool CommonChassisDriver::computeKinematicTwist(
  const HardwareFeedback& feedback,
  double& vx, double& vy, double& wz) const
{
  if (!allFinite(feedback.wheel_speeds) ||
      !allFinite(feedback.steering_angles)) {
    return false;
  }

  std::vector<double> speeds = feedback.wheel_speeds;
  for (double& speed : speeds) {
    speed *= odom_linear_scale_;
  }

  if (odometry_model_ == OdometryModel::DIFFERENTIAL) {
    if (speeds.size() != 2 || !feedback.steering_angles.empty()) {
      return false;
    }
    vx = 0.5 * (speeds[0] + speeds[1]);
    vy = 0.0;
    wz = (speeds[1] - speeds[0]) / wheel_separation_;
    return true;
  }

  if (odometry_model_ == OdometryModel::DOUBLE_DIFFERENTIAL) {
    if (speeds.size() != 4 || feedback.steering_angles.size() != 2) {
      return false;
    }
    const std::vector<double> group_speeds{
      0.5 * (speeds[0] + speeds[1]),
      0.5 * (speeds[2] + speeds[3])};
    return solveWheelConstraints(
      group_speeds, feedback.steering_angles,
      {0.5 * wheelbase_, -0.5 * wheelbase_}, {0.0, 0.0},
      false, vx, vy, wz);
  }

  if (odometry_model_ == OdometryModel::SINGLE_STEER) {
    if (speeds.size() != 1 || feedback.steering_angles.size() != 1) {
      return false;
    }
    return solveWheelConstraints(
      speeds, feedback.steering_angles,
      {wheelbase_}, {steer_offset_y_}, true, vx, vy, wz);
  }

  if (odometry_model_ == OdometryModel::DOUBLE_STEER) {
    if (speeds.size() != 2 || feedback.steering_angles.size() != 2) {
      return false;
    }
    return solveWheelConstraints(
      speeds, feedback.steering_angles,
      {0.5 * wheelbase_, -0.5 * wheelbase_},
      {0.5 * wheel_distance_, -0.5 * wheel_distance_},
      false, vx, vy, wz);
  }

  if (odometry_model_ == OdometryModel::FOUR_STEER) {
    if (speeds.size() != 4 || feedback.steering_angles.size() != 4) {
      return false;
    }
    return solveWheelConstraints(
      speeds, feedback.steering_angles,
      wheel_positions_x_, wheel_positions_y_, false, vx, vy, wz);
  }

  if (odometry_model_ == OdometryModel::GENERAL_KINEMATICS) {
    if (speeds.size() != static_cast<size_t>(num_wheels_) ||
        feedback.steering_angles.size() != static_cast<size_t>(num_wheels_)) {
      return false;
    }
    return solveWheelConstraints(
      speeds, feedback.steering_angles,
      wheel_positions_x_, wheel_positions_y_, num_wheels_ == 1,
      vx, vy, wz);
  }
  return false;
}

bool CommonChassisDriver::solveWheelConstraints(
  const std::vector<double>& speeds,
  const std::vector<double>& angles,
  const std::vector<double>& positions_x,
  const std::vector<double>& positions_y,
  bool constrain_lateral,
  double& vx, double& vy, double& wz) const
{
  const size_t count = speeds.size();
  if (count == 0 || angles.size() != count ||
      positions_x.size() != count || positions_y.size() != count ||
      !allFinite(speeds) || !allFinite(angles) ||
      !allFinite(positions_x) || !allFinite(positions_y)) {
    return false;
  }

  std::array<std::array<double, 3>, 3> normal{};
  std::array<double, 3> rhs{};
  const auto add_row = [&normal, &rhs](
    const std::array<double, 3>& row, double value) {
      for (size_t i = 0; i < 3; ++i) {
        rhs[i] += row[i] * value;
        for (size_t j = 0; j < 3; ++j) {
          normal[i][j] += row[i] * row[j];
        }
      }
    };

  for (size_t i = 0; i < count; ++i) {
    add_row({1.0, 0.0, -positions_y[i]},
      speeds[i] * std::cos(angles[i]));
    add_row({0.0, 1.0, positions_x[i]},
      speeds[i] * std::sin(angles[i]));
  }
  if (constrain_lateral) {
    add_row({0.0, 1.0, 0.0}, 0.0);
  }
  for (size_t i = 0; i < 3; ++i) {
    normal[i][i] += kinematic_damping_;
  }

  std::array<double, 3> solution{};
  if (!solveSymmetric3x3(normal, rhs, solution)) {
    return false;
  }
  vx = solution[0];
  vy = solution[1];
  wz = solution[2];
  return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz);
}

double CommonChassisDriver::encoderDelta(
  int64_t current, int64_t previous) const
{
  double delta = static_cast<double>(current) - static_cast<double>(previous);
  if (encoder_bits_ == 0) {
    return delta;
  }

  const double range = std::ldexp(1.0, encoder_bits_);
  const double half_range = 0.5 * range;
  if (delta > half_range) {
    delta -= range;
  } else if (delta < -half_range) {
    delta += range;
  }
  return delta;
}

void CommonChassisDriver::clearOdometryLocked()
{
  state_.x = 0.0;
  state_.y = 0.0;
  state_.yaw = 0.0;
  state_.linear_vel = 0.0;
  state_.linear_vel_y = 0.0;
  state_.angular_vel = 0.0;
  state_.left_encoder = 0;
  state_.right_encoder = 0;
  state_.error_code = 0;
  state_.status_msg.clear();
  feedback_initialized_ = false;
  direct_origin_valid_ = false;
  direct_origin_pending_ = false;
  last_direct_x_ = 0.0;
  last_direct_y_ = 0.0;
  last_direct_yaw_ = 0.0;
}

}  // namespace chassis_interface
