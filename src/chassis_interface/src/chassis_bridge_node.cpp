#include <chassis_interface/chassis_bridge.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chassis_bridge_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  chassis_interface::ChassisBridge bridge;
  if (!bridge.init(nh, pnh))
  {
    ROS_FATAL("[chassis_bridge_node] Failed to initialize. Exiting.");
    return 1;
  }

  ros::spin();
  return 0;
}
