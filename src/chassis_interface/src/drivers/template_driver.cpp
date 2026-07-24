#include <chassis_interface/common_chassis_driver.h>

#include <pluginlib/class_list_macros.h>

#include <array>
#include <mutex>
#include <vector>

namespace chassis_interface
{

/**
 * 新底盘最小接入模板。
 *
 * 当前模板使用 VelocityOdometryDriver，并把最后一次控制指令回环成速度反馈，
 * 因此不依赖真实硬件即可运行。接入新底盘时：
 *   1. 复制本文件并修改类名；
 *   2. 根据硬件反馈类型选择对应的里程计父类；
 *   3. 实现 sendCommandToHardware() 和 receiveStateFromHardware()；
 *   4. 在 plugin XML、CMakeLists.txt 和 YAML 中注册新驱动。
 *
 * 文件下半部分给出了全部九种里程计父类的中文案例。
 */
class TemplateDriver : public VelocityOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    // TODO：替换为串口、CAN 或 TCP 的控制指令编码与发送。
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_command_ = cmd;
    return true;
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    // TODO：替换为非阻塞硬件读取及协议解析。
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback.stamp = ros::Time::now();
    feedback.linear_vel = last_command_.linear_vel;
    feedback.angular_vel = last_command_.angular_vel;
    feedback.is_connected = true;
    return true;
  }

private:
  std::mutex feedback_mutex_;
  ChassisCommand last_command_;
};

}  // namespace chassis_interface

PLUGINLIB_EXPORT_CLASS(
  chassis_interface::TemplateDriver, chassis_interface::ChassisDriver)

/*
 * ============================================================================
 * 各类里程计接入案例
 * ============================================================================
 *
 * 以下代码放在 #if 0 中，仅作为可复制的开发模板，不参与当前插件编译和注册。
 *
 * 所有案例共同约定：
 *   - receiveStateFromHardware() 暂时没有新数据时返回 false，不能长时间阻塞；
 *   - 收到有效数据时填写 feedback.stamp 和 feedback.is_connected；
 *   - wheel_speeds 单位为 m/s，正方向沿轮子实际滚动方向；
 *   - steering_angles 单位为 rad，是车体坐标系中的实际滚动方向；
 *   - 车体坐标系 X 向前、Y 向左，逆时针角度为正；
 *   - 电机 RPM、减速比、轮径、安装角和编码器零偏应在协议驱动中完成换算。
 */
#if 0

namespace chassis_interface
{

// ============================================================================
// 案例 1：左右轮累计编码器差速里程
// 父类：WheelOdometryDriver
// 必填：left_encoder、right_encoder、stamp
// YAML：wheel_separation、wheel_radius、encoder_ticks_per_rev、encoder_bits
// ============================================================================
class EncoderWheelExampleDriver : public WheelOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    // 将车体 v/w 转换成底盘协议并发送。
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    int64_t left_count = 0;
    int64_t right_count = 0;
    if (!protocol_.tryReadEncoder(left_count, right_count)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.left_encoder = left_count;
    feedback.right_encoder = right_count;
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 2：车体线速度/角速度积分里程
// 父类：VelocityOdometryDriver
// 必填：linear_vel、angular_vel、stamp
// 适用：下位机直接反馈车体 vx 和 wz，但不反馈绝对位姿
// ============================================================================
class VelocityExampleDriver : public VelocityOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    if (!protocol_.tryReadBodyVelocity(
        feedback.linear_vel, feedback.angular_vel)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 3：下位机绝对位姿里程
// 父类：DirectOdometryDriver
// 必填：x、y、yaw、stamp
// 可选：同时填写 linear_vel、angular_vel，并将 velocity_valid 设为 true
// ============================================================================
class DirectPoseExampleDriver : public DirectOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    if (!protocol_.tryReadPose(feedback.x, feedback.y, feedback.yaw)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.velocity_valid = false;  // 公共层根据相邻位姿计算速度。
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 4：左右轮线速度差速里程
// 父类：DifferentialOdometryDriver
// wheel_speeds 顺序：[左轮, 右轮]
// steering_angles：必须为空
// YAML：wheel_separation
// ============================================================================
class DifferentialExampleDriver : public DifferentialOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    double left_speed = 0.0;
    double right_speed = 0.0;
    if (!protocol_.tryReadWheelSpeed(left_speed, right_speed)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.wheel_speeds = {left_speed, right_speed};
    feedback.steering_angles.clear();
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 5：前后双差速轮组里程
// 父类：DoubleDifferentialOdometryDriver
// wheel_speeds 顺序：[前左, 前右, 后左, 后右]
// steering_angles 顺序：[前轮组舵角, 后轮组舵角]
// YAML：wheelbase
// ============================================================================
class DoubleDifferentialExampleDriver
  : public DoubleDifferentialOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    std::array<double, 4> speeds{};
    std::array<double, 2> angles{};
    if (!protocol_.tryReadDoubleDifferential(speeds, angles)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.wheel_speeds.assign(speeds.begin(), speeds.end());
    feedback.steering_angles.assign(angles.begin(), angles.end());
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 6：单舵轮里程
// 父类：SingleSteerOdometryDriver
// wheel_speeds 顺序：[舵轮线速度]
// steering_angles 顺序：[舵轮实际舵角]
// YAML：wheelbase、steer_offset_y
// ============================================================================
class SingleSteerExampleDriver : public SingleSteerOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    double wheel_speed = 0.0;
    double steer_angle = 0.0;
    if (!protocol_.tryReadSingleSteer(wheel_speed, steer_angle)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.wheel_speeds = {wheel_speed};
    feedback.steering_angles = {steer_angle};
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 7：前后双舵轮里程
// 父类：DoubleSteerOdometryDriver
// wheel_speeds 顺序：[前舵轮, 后舵轮]
// steering_angles 顺序：[前舵角, 后舵角]
// YAML：wheelbase、wheel_distance
// ============================================================================
class DoubleSteerExampleDriver : public DoubleSteerOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    std::array<double, 2> speeds{};
    std::array<double, 2> angles{};
    if (!protocol_.tryReadDoubleSteer(speeds, angles)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.wheel_speeds.assign(speeds.begin(), speeds.end());
    feedback.steering_angles.assign(angles.begin(), angles.end());
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 8：四舵轮里程
// 父类：FourSteerOdometryDriver
// wheel_speeds 和 steering_angles 均按轮 1、2、3、4 排列
// YAML：wheel_positions_x、wheel_positions_y，各数组必须包含 4 个元素
// ============================================================================
class FourSteerExampleDriver : public FourSteerOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    std::array<double, 4> speeds{};
    std::array<double, 4> angles{};
    if (!protocol_.tryReadFourSteer(speeds, angles)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.wheel_speeds.assign(speeds.begin(), speeds.end());
    feedback.steering_angles.assign(angles.begin(), angles.end());
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

// ============================================================================
// 案例 9：1～4 轮任意布局通用运动学里程
// 父类：GeneralKinematicsOdometryDriver
// wheel_speeds 和 steering_angles 数量必须等于 num_wheels
// YAML：num_wheels、wheel_positions_x、wheel_positions_y
// 注意：通用单轮模型会自动施加车体横向速度 vy=0 的约束
// ============================================================================
class GeneralKinematicsExampleDriver
  : public GeneralKinematicsOdometryDriver
{
protected:
  bool sendCommandToHardware(const ChassisCommand& cmd) override
  {
    return protocol_.sendVelocity(cmd.linear_vel, cmd.angular_vel);
  }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    std::vector<double> speeds;
    std::vector<double> angles;
    if (!protocol_.tryReadWheels(speeds, angles)) {
      return false;
    }
    feedback.stamp = ros::Time::now();
    feedback.wheel_speeds = std::move(speeds);
    feedback.steering_angles = std::move(angles);
    feedback.is_connected = true;
    return true;
  }

private:
  MyProtocol protocol_;
};

}  // namespace chassis_interface

#endif  // 仅作为接入案例，不参与编译
