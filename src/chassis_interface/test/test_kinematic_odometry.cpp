#include <chassis_interface/common_chassis_driver.h>

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace chassis_interface
{
namespace
{

template<typename Base>
class FakeDriver : public Base
{
public:
  void push(HardwareFeedback feedback)
  {
    feedback.is_connected = true;
    feedback_queue_.push_back(std::move(feedback));
  }

protected:
  bool sendCommandToHardware(const ChassisCommand&) override { return true; }

  bool receiveStateFromHardware(HardwareFeedback& feedback) override
  {
    if (feedback_queue_.empty()) {
      return false;
    }
    feedback = std::move(feedback_queue_.front());
    feedback_queue_.pop_front();
    return true;
  }

private:
  std::deque<HardwareFeedback> feedback_queue_;
};

class KinematicOdometryTest : public ::testing::Test
{
protected:
  template<typename Driver>
  std::shared_ptr<Driver> makeDriver(
    const std::function<void(ros::NodeHandle&)>& configure = {})
  {
    ros::NodeHandle node(
      "~kinematic_odom_test_" + std::to_string(next_node_id_++));
    node.setParam("feedback_timeout", 0.0);
    node.setParam("max_feedback_dt", 2.0);
    if (configure) {
      configure(node);
    }
    auto driver = std::make_shared<Driver>();
    EXPECT_TRUE(driver->connect(node));
    return driver;
  }

  static HardwareFeedback sample(
    int seconds,
    std::vector<double> speeds,
    std::vector<double> angles = {})
  {
    HardwareFeedback feedback;
    feedback.stamp = ros::Time(seconds, 0);
    feedback.wheel_speeds = std::move(speeds);
    feedback.steering_angles = std::move(angles);
    return feedback;
  }

  template<typename Driver>
  static ChassisState integrateTwice(
    Driver& driver,
    const std::vector<double>& speeds,
    const std::vector<double>& angles = {})
  {
    driver.push(sample(1, speeds, angles));
    (void)driver.readState();
    driver.push(sample(2, speeds, angles));
    return driver.readState();
  }

  static int next_node_id_;
};

int KinematicOdometryTest::next_node_id_ = 0;

TEST_F(KinematicOdometryTest, WheelEncoderIntegratesDistance)
{
  auto driver = makeDriver<FakeDriver<WheelOdometryDriver>>(
    [](ros::NodeHandle& node) {
      node.setParam("wheel_separation", 1.0);
      node.setParam("wheel_radius", 1.0 / (2.0 * M_PI));
      node.setParam("encoder_ticks_per_rev", 1);
      node.setParam("encoder_bits", 0);
    });

  HardwareFeedback first;
  first.stamp = ros::Time(1, 0);
  driver->push(first);
  (void)driver->readState();
  HardwareFeedback second;
  second.stamp = ros::Time(2, 0);
  second.left_encoder = 1;
  second.right_encoder = 1;
  driver->push(second);
  const ChassisState state = driver->readState();

  EXPECT_NEAR(state.x, 1.0, 1e-9);
  EXPECT_NEAR(state.linear_vel, 1.0, 1e-9);
}

TEST_F(KinematicOdometryTest, VelocityFeedbackIntegratesBodyTwist)
{
  auto driver = makeDriver<FakeDriver<VelocityOdometryDriver>>();
  HardwareFeedback feedback;
  feedback.stamp = ros::Time(1, 0);
  feedback.linear_vel = 1.0;
  driver->push(feedback);
  (void)driver->readState();
  feedback.stamp = ros::Time(2, 0);
  driver->push(feedback);
  const ChassisState state = driver->readState();

  EXPECT_NEAR(state.x, 1.0, 1e-9);
  EXPECT_NEAR(state.linear_vel, 1.0, 1e-9);
}

TEST_F(KinematicOdometryTest, DirectPoseDerivesLateralVelocity)
{
  auto driver = makeDriver<FakeDriver<DirectOdometryDriver>>();
  HardwareFeedback feedback;
  feedback.stamp = ros::Time(1, 0);
  driver->push(feedback);
  (void)driver->readState();
  feedback.stamp = ros::Time(2, 0);
  feedback.y = 1.0;
  driver->push(feedback);
  const ChassisState state = driver->readState();

  EXPECT_NEAR(state.y, 1.0, 1e-9);
  EXPECT_NEAR(state.linear_vel, 0.0, 1e-9);
  EXPECT_NEAR(state.linear_vel_y, 1.0, 1e-9);
}

TEST_F(KinematicOdometryTest, DifferentialStraightAndTurn)
{
  auto driver = makeDriver<FakeDriver<DifferentialOdometryDriver>>(
    [](ros::NodeHandle& node) { node.setParam("wheel_separation", 1.0); });

  driver->push(sample(1, {1.0, 1.0}));
  (void)driver->readState();
  driver->push(sample(2, {0.0, 2.0}));
  const ChassisState state = driver->readState();

  EXPECT_NEAR(state.linear_vel, 1.0, 1e-9);
  EXPECT_NEAR(state.linear_vel_y, 0.0, 1e-9);
  EXPECT_NEAR(state.angular_vel, 2.0, 1e-9);
  EXPECT_NEAR(state.yaw, 2.0, 1e-9);
  EXPECT_NEAR(state.x, std::cos(1.0), 1e-9);
  EXPECT_NEAR(state.y, std::sin(1.0), 1e-9);
}

TEST_F(KinematicOdometryTest, DoubleDifferentialStraight)
{
  auto driver = makeDriver<FakeDriver<DoubleDifferentialOdometryDriver>>(
    [](ros::NodeHandle& node) { node.setParam("wheelbase", 1.0); });
  const ChassisState state = integrateTwice(
    *driver, {1.0, 1.0, 1.0, 1.0}, {0.0, 0.0});

  EXPECT_NEAR(state.x, 1.0, 1e-8);
  EXPECT_NEAR(state.y, 0.0, 1e-8);
  EXPECT_NEAR(state.angular_vel, 0.0, 1e-8);
}

TEST_F(KinematicOdometryTest, SingleSteerUsesWheelbaseAndOffset)
{
  auto driver = makeDriver<FakeDriver<SingleSteerOdometryDriver>>(
    [](ros::NodeHandle& node) {
      node.setParam("wheelbase", 2.0);
      node.setParam("steer_offset_y", 0.5);
    });
  const ChassisState state = integrateTwice(
    *driver, {1.0}, {M_PI / 2.0});

  EXPECT_NEAR(state.linear_vel, 0.25, 1e-8);
  EXPECT_NEAR(state.linear_vel_y, 0.0, 1e-8);
  EXPECT_NEAR(state.angular_vel, 0.5, 1e-8);
}

TEST_F(KinematicOdometryTest, DoubleSteerPublishesLateralVelocity)
{
  auto driver = makeDriver<FakeDriver<DoubleSteerOdometryDriver>>(
    [](ros::NodeHandle& node) {
      node.setParam("wheelbase", 1.0);
      node.setParam("wheel_distance", 0.0);
    });
  const ChassisState state = integrateTwice(
    *driver, {1.0, 1.0}, {M_PI / 2.0, M_PI / 2.0});

  EXPECT_NEAR(state.x, 0.0, 1e-8);
  EXPECT_NEAR(state.y, 1.0, 1e-8);
  EXPECT_NEAR(state.linear_vel_y, 1.0, 1e-8);
  EXPECT_NEAR(state.angular_vel, 0.0, 1e-8);
}

TEST_F(KinematicOdometryTest, FourSteerPublishesLateralVelocity)
{
  auto driver = makeDriver<FakeDriver<FourSteerOdometryDriver>>();
  const ChassisState state = integrateTwice(
    *driver, {1.0, 1.0, 1.0, 1.0},
    {M_PI / 2.0, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0});

  EXPECT_NEAR(state.x, 0.0, 1e-8);
  EXPECT_NEAR(state.y, 1.0, 1e-8);
  EXPECT_NEAR(state.linear_vel_y, 1.0, 1e-8);
  EXPECT_NEAR(state.angular_vel, 0.0, 1e-8);
}

TEST_F(KinematicOdometryTest, GeneralOneWheelAddsNonholonomicConstraint)
{
  auto driver = makeDriver<FakeDriver<GeneralKinematicsOdometryDriver>>(
    [](ros::NodeHandle& node) {
      node.setParam("num_wheels", 1);
      node.setParam("wheel_positions_x", std::vector<double>{2.0});
      node.setParam("wheel_positions_y", std::vector<double>{0.0});
    });
  const ChassisState state = integrateTwice(
    *driver, {1.0}, {M_PI / 2.0});

  EXPECT_NEAR(state.linear_vel, 0.0, 1e-8);
  EXPECT_NEAR(state.linear_vel_y, 0.0, 1e-8);
  EXPECT_NEAR(state.angular_vel, 0.5, 1e-8);
}

TEST_F(KinematicOdometryTest, RejectsInvalidFeedbackAndResetClearsState)
{
  auto driver = makeDriver<FakeDriver<DifferentialOdometryDriver>>(
    [](ros::NodeHandle& node) { node.setParam("wheel_separation", 1.0); });
  (void)integrateTwice(*driver, {1.0, 1.0});

  driver->push(sample(3, {1.0}));
  ChassisState state = driver->readState();
  EXPECT_EQ(state.error_code, 4);
  EXPECT_NEAR(state.x, 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.linear_vel, 0.0);

  driver->resetOdom();
  state = driver->readState();
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);
  EXPECT_DOUBLE_EQ(state.linear_vel_y, 0.0);
}

TEST_F(KinematicOdometryTest, RejectsNonFiniteAndOversizedTimeStep)
{
  auto driver = makeDriver<FakeDriver<DifferentialOdometryDriver>>(
    [](ros::NodeHandle& node) { node.setParam("wheel_separation", 1.0); });

  driver->push(sample(1, {1.0, 1.0}));
  (void)driver->readState();
  driver->push(sample(4, {1.0, 1.0}));
  ChassisState state = driver->readState();
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.linear_vel, 0.0);

  driver->push(sample(
    5, {std::numeric_limits<double>::quiet_NaN(), 1.0}));
  state = driver->readState();
  EXPECT_EQ(state.error_code, 4);
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.angular_vel, 0.0);
}

}  // namespace
}  // namespace chassis_interface

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_kinematic_odometry");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
