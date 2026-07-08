#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

class ImuYawPublisher
{
public:
  ImuYawPublisher()
  {
    ros::NodeHandle nh;
    imu_sub_ = nh.subscribe("/imu", 10, &ImuYawPublisher::imuCallback, this);
    yaw_pub_ = nh.advertise<geometry_msgs::Vector3>("/imu_yaw", 10);
  }

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    double x = msg->orientation.x;
    double y = msg->orientation.y;
    double z = msg->orientation.z;
    double w = msg->orientation.w;

    // 四元数 → Roll/Pitch/Yaw
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    double roll  = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    double pitch = std::asin(std::max(-1.0, std::min(1.0, sinp)));

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    geometry_msgs::Vector3 yaw_msg;
    yaw_msg.x = roll;
    yaw_msg.y = pitch;
    yaw_msg.z = yaw;
    yaw_pub_.publish(yaw_msg);
  }

  ros::Subscriber imu_sub_;
  ros::Publisher yaw_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_yaw_publisher");
  ImuYawPublisher node;
  ros::spin();
  return 0;
}
