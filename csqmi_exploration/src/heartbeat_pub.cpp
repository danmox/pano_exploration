#include <ros/ros.h>
#include <std_msgs/Header.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heartbeat_publisher");
  ros::NodeHandle nh;

  ros::Publisher stamp_pub = nh.advertise<std_msgs::Header>("heartbeat", 10);

  ros::Rate rate(10);
  while (ros::ok()) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    stamp_pub.publish(header);
    rate.sleep();
  }

  return 0;
}
