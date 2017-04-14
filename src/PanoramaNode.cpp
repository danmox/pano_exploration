#include <ros/ros.h>
#include <panorama/Panorama.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panorama_action_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string name = "panorama_action_server";
  panorama::Panorama server(nh, pnh, name);

  ros::waitForShutdown();

  return 0;
}
