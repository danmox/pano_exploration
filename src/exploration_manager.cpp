#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>
#include <geometry_msgs/Twist.h>

#include <string.h>

typedef actionlib::SimpleActionClient<panorama::PanoramaAction> PanAC;

int main(int argc, char** argv)
{
  /*
   * initialize ROS components
   */

  ros::init(argc, argv, "csqmi_exploration_manager");
  ros::NodeHandle nh, pnh("~");

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("vel_cmds", 10);

  std::string tf_prefix;
  if (!pnh.getParam("tf_prefix", tf_prefix)) {
    ROS_FATAL("main(...): failed to read params from server");
    exit(EXIT_FAILURE);
  }

  /*
  string server_name = tf_prefix + string(argv[1]) + "/panorama_action_server";
  ActionClient ac(server_name.c_str(), true);
  ROS_INFO("Waiting for action server to start: %s\n", server_name.c_str());
  ac.waitForServer();
  ROS_INFO("%s is ready\n", server_name.c_str());
  */

  /*
   * Make robot complete a small circle to help initialize SLAM and relative
   * localization (for the multi-robot case)
   */

  double v = 0.15; // m/s
  double r = 0.5; // m
  double w = v / r;
  geometry_msgs::Twist turn_cmd;
  turn_cmd.linear.x = v;
  turn_cmd.angular.z = w;
  double t = round(2.0 * M_PI * r / v);
  ros::Rate loop_rate(10);
  int loops = 0;
  while (ros::ok() && loops < (int)t*10) {
    vel_pub.publish(turn_cmd);
    loop_rate.sleep();
    ++loops;
  }

  /*
  PanoramaGoal ag;
  ag.file_name = "robot" + string(argv[1]) + "pan" + to_string(pan++);
  ac.sendGoal(ag, &doneCB, &activeCB, &feedbackCB);
  ac.waitForResult();
  */

  return 0;
}
