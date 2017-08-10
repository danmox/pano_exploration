#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <grid_mapping/angle_grid.h>
#include <csqmi_planning/csqmi_planning.h>

#include <opencv2/opencv.hpp>
#include <string.h>
#include <vector>

typedef actionlib::SimpleActionClient<panorama::PanoramaAction> PanAC;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveAC;

std::string pan_file;

//
// panorama action functions
//

void panFeedbackCB(const panorama::PanoramaFeedbackConstPtr& feedback)
{
  ROS_INFO("Captured frame %d of 72", feedback->frames_captured);
}

void panActiveCB()
{
  ROS_INFO("Capturing panorama...");
}

void panDoneCB(const actionlib::SimpleClientGoalState& state,
    const panorama::PanoramaResultConstPtr& result)
{
  ROS_INFO("Panorama completed with %s and saved to %s", 
      state.toString().c_str(), result->full_file_name.c_str());
  if (state.toString().compare("SUCCEEDED") == 0)
    pan_file = result->full_file_name;
}

//
// move_base action functions
//

void moveFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr& msg)
{
}

void moveActiveCB()
{
  ROS_INFO("Navigating to goal...");
}

void moveDoneCB(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Navigation completed with status: %s", state.toString().c_str());
}

//
// main(...)
//

int main(int argc, char** argv)
{
  /*
   * initialize ROS components
   */

  ros::init(argc, argv, "csqmi_exploration_manager");
  ros::NodeHandle nh, pnh("~");

  ros::Publisher vel_pub, viz_map_pub, pan_grid_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("velocity_commands", 10);
  viz_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_2D", 2);
  pan_grid_pub = nh.advertise<grid_mapping::OccupancyGrid>("angle_grid", 2);

  std::string tf_prefix;
  double scan_range_min, scan_range_max;
  if (!pnh.getParam("tf_prefix", tf_prefix) ||
      !nh.getParam("/scan_range_min", scan_range_min) ||
      !nh.getParam("/scan_range_max", scan_range_max)) {
    ROS_FATAL("main(...): failed to read params from server");
    exit(EXIT_FAILURE);
  }

  /*
   * Initialize action servers for panorama capture and navigation
   */

  std::string pan_server_name = "/" + tf_prefix + "/panorama_action_server";
  PanAC pan_ac(pan_server_name.c_str(), true);
  ROS_INFO("Waiting for action server to start: %s", pan_server_name.c_str());
  pan_ac.waitForServer();
  ROS_INFO("%s is ready", pan_server_name.c_str());

  std::string nav_server_name = "/" + tf_prefix + "/move_base";
  MoveAC move_ac(nav_server_name.c_str(), true);
  ROS_INFO("Waiting for action server to start: %s", nav_server_name.c_str());
  move_ac.waitForServer();
  ROS_INFO("%s is ready", nav_server_name.c_str());

  /*
   * Give the other components/robots time to load before staring exploration
   */

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

  int pan_count = 0;
  grid_mapping::AngleGrid ang_grid(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1);
  ang_grid.range_min = scan_range_min;
  ang_grid.range_max = scan_range_max;
  std::vector<cv::Point> pan_px;
  while (ros::ok()) {

    /*
     * Collect panorama, insert it into the angle grid, and publish the angle grid
     * and 2D rviz version
     */

    panorama::PanoramaGoal pan_goal;
    pan_goal.file_name = tf_prefix + "pan" + std::to_string(++pan_count);
    pan_ac.sendGoal(pan_goal, &panDoneCB, &panActiveCB, &panFeedbackCB);
    pan_ac.waitForResult();
    if (pan_file.size() == 0) {
      ROS_WARN("Panorama action returned no file. Restarting");
      --pan_count;
      continue;
    }

    // create grid of just the panorama and publish
    grid_mapping::AngleGrid pan_grid(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1);
    pan_grid.range_min = scan_range_min;
    pan_grid.range_max = scan_range_max;
    pan_grid.insertPanorama(pan_file);
    pan_grid_pub.publish(pan_grid.createROSMsg());

    // insert the pan grid into ang grid and publish for visualization
    ang_grid.insertPanorama(pan_file);
    viz_map_pub.publish(ang_grid.createROSOGMsg());

    /*
     * Compute next panorama capture location
     */

    // read panoaram capture location
    rosbag::Bag panbag;
    panbag.open(pan_file, rosbag::bagmode::Read);
    geometry_msgs::TransformStamped pan_pose;
    for (auto m : rosbag::View(panbag, rosbag::TopicQuery("panorama_pose"))) {
      auto msg = m.instantiate<geometry_msgs::TransformStamped>();
      if (msg) {
        pan_pose = *msg;
        break;
      }
    }
    grid_mapping::Point panorama_position(pan_pose.transform.translation.x,
        pan_pose.transform.translation.y);

    // compute skeleton
    cv::Mat skel;
    computeSkeleton(ang_grid, skel, 2);

    // partition skel
    cv::Point po;
    ang_grid.positionToSubscripts(panorama_position, po.y, po.x);
    pan_px.push_back(po);
    std::vector<std::vector<cv::Point>> regions = partitionSkeleton(skel, pan_px);

    // find the point of each region with the hightest CSQMI
    CSQMI objective(DepthCamera(180, scan_range_max, scan_range_min), 0.03);
    std::vector<cv::Point> max_csqmi_pts;
    std::vector<double> region_max_csqmi;
    for (auto& reg : regions) {
      std::vector<double> reg_mi = objective.csqmi(ang_grid, reg);
      int idx = std::max_element(reg_mi.begin(), reg_mi.end()) - reg_mi.begin();
      max_csqmi_pts.push_back(reg[idx]);
      region_max_csqmi.push_back(reg_mi[idx]);
    }

    // choose the goal point
    auto goal_it = std::max_element(region_max_csqmi.begin(), region_max_csqmi.end());
    cv::Point goal_px = max_csqmi_pts[goal_it - region_max_csqmi.begin()];
    grid_mapping::Point goal_pt = ang_grid.subscriptsToPosition(goal_px.y, goal_px.x);
    move_base_msgs::MoveBaseGoal action_goal;
    action_goal.target_pose.pose.position.x = goal_pt.x;
    action_goal.target_pose.pose.position.y = goal_pt.y;
    action_goal.target_pose.header.stamp = ros::Time::now();
    action_goal.target_pose.header.frame_id = "/" + tf_prefix + "/map";
    action_goal.target_pose.pose.orientation.w = 1.0;

    /*
     * Navigate to the goal point
     */

    move_ac.sendGoal(action_goal, &moveDoneCB, &moveActiveCB, &moveFeedbackCB);
    move_ac.waitForResult();

    ROS_INFO("Completed %d iteration loop", pan_count);
  }

  return 0;
}
