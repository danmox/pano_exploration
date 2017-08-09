#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>
#include <geometry_msgs/Twist.h>
#include <grid_mapping/angle_grid.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string.h>

typedef actionlib::SimpleActionClient<panorama::PanoramaAction> PanAC;

void feedbackCB(const panorama::PanoramaFeedbackConstPtr& feedback)
{
  ROS_INFO("Captured frame %d of 72", feedback->frames_captured);
}

void activeCB()
{
  ROS_INFO("Capturing panorama...");
}

std::string pan_file;
void doneCB(const actionlib::SimpleClientGoalState& state,
            const panorama::PanoramaResultConstPtr& result)
{
  ROS_INFO("Panorama completed with %s and saved to %s", 
      state.toString().c_str(), result->full_file_name.c_str());
  if (state.toString().compare("SUCCEEDED") == 0)
    pan_file = result->full_file_name;
}

int main(int argc, char** argv)
{
  /*
   * initialize ROS components
   */

  ros::init(argc, argv, "csqmi_exploration_manager");
  ros::NodeHandle nh, pnh("~");

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("vel_cmds", 10);
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

  std::string tf_prefix;
  if (!pnh.getParam("tf_prefix", tf_prefix)) {
    ROS_FATAL("main(...): failed to read params from server");
    exit(EXIT_FAILURE);
  }

  std::string server_name = "/" + tf_prefix + "/panorama_action_server";
  PanAC ac(server_name.c_str(), true);
  ROS_INFO("Waiting for action server to start: %s", server_name.c_str());
  ac.waitForServer();
  ROS_INFO("%s is ready", server_name.c_str());

  ros::Rate countdown(1);
  for (int i = 10; i > 0; --i) {
    ROS_INFO("Beginning exploration in %d seconds...", i);
    countdown.sleep();
  }

  /*
   * Make robot complete a small circle to help initialize SLAM and relative
   * localization (for the multi-robot case)
   */

  ROS_INFO("Running robot in a small circle");
  double v = 0.1; // m/s
  double r = 0.3; // m
  double w = v / r;
  geometry_msgs::Twist turn_cmd;
  turn_cmd.linear.x = v;
  turn_cmd.angular.z = w;
  double t = 2.0 * M_PI * r / v;
  ros::Rate loop_rate(10);
  ros::Time start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < t + 1.0) {
    vel_pub.publish(turn_cmd);
    loop_rate.sleep();
  }

  panorama::PanoramaGoal ag;
  ag.file_name = tf_prefix + "pan" + std::to_string(1);
  ac.sendGoal(ag, &doneCB, &activeCB, &feedbackCB);
  ac.waitForResult();

  grid_mapping::AngleGrid ang_grid(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1);
  if (pan_file.size() != 0)
    ang_grid.insertPanorama(pan_file);
  else
    ROS_ERROR("pan_file is empty");

  nav_msgs::OccupancyGridPtr grid_msg = ang_grid.createROSOGMsg();
  map_pub.publish(grid_msg);

  ros::spin();

  return 0;
}
