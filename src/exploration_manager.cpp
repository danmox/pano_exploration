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
#include <sensor_msgs/PointCloud2.h>
#include <grid_mapping/OccupancyGrid.h>

#include <grid_mapping/angle_grid.h>
#include <csqmi_planning/csqmi_planning.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <string.h>
#include <sstream>
#include <vector>
#include <algorithm>

using grid_mapping::AngleGrid;
using sensor_msgs::PointCloud2;
using std::vector;
using std::string;

typedef actionlib::SimpleActionClient<panorama::PanoramaAction> PanAC;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveAC;

typedef vector<vector<cv::Point>> regions_vec;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

struct GoalIDPair
{
  grid_mapping::Point point;
  int id;
};

//
// global variables
//

static int robot_id;
static bool received_first_map = false;
static double robot1_y_offset;
static string tf_prefix;
static string pan_file;
static AngleGrid ang_grid(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1);
static ros::Publisher viz_map_pub;
static vector<grid_mapping::Point> pan_locations;
static vector<GoalIDPair> goals;
static tf2_ros::Buffer tfBuffer;

//
// panorama action functions
//

void panFeedbackCB(const panorama::PanoramaFeedbackConstPtr& feedback)
{
  //ROS_INFO("Captured frame %d of 72", feedback->frames_captured);
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
// visualization functions
//

pcl::PointXYZRGB pixelToPCLPoint(const AngleGrid& grid, cv::Point px, int r,
    int g, int b)
{
  grid_mapping::Point pt = grid.subscriptsToPosition(px.y, px.x);
  pcl::PointXYZRGB pcl_point(r,g,b);
  pcl_point.x = pt.x;
  pcl_point.y = pt.y;
  return pcl_point;
}

CloudXYZRGB skeletonToPC(const AngleGrid& grid, const regions_vec& regions,
    int r, int g, int b)
{
  CloudXYZRGB skel_cloud;
  for (auto& region : regions) {
    for (auto& px : region) {
      grid_mapping::Point pt = grid.subscriptsToPosition(px.y, px.x);
      pcl::PointXYZRGB pcl_point(r,g,b);
      pcl_point.x = pt.x;
      pcl_point.y = pt.y;
      skel_cloud.points.push_back(pcl_point);
    }
  }
  skel_cloud.width = skel_cloud.points.size();
  skel_cloud.height = 1;
  skel_cloud.is_dense = false;
  return skel_cloud;
}

void addPixelToPC(const AngleGrid& grid, CloudXYZRGB& pc, cv::Point px, int r,
    int g, int b)
{
  grid_mapping::Point pt = grid.subscriptsToPosition(px.y, px.x);
  pcl::PointXYZRGB pcl_point(r,g,b);
  pcl_point.x = pt.x;
  pcl_point.y = pt.y;
  pc.points.push_back(pcl_point);
  pc.width++;
}

void addPixelsToPC(const AngleGrid& grid, CloudXYZRGB& pc,
    vector<cv::Point>& pxs, int r, int g, int b)
{
  for (auto& px : pxs)
    addPixelToPC(grid, pc, px, r, g, b);
}

void addPixelsToPC(const AngleGrid& grid, CloudXYZRGB& pc,
    vector<InfoPxPair>& info_px_pairs, int r, int g, int b)
{
  for (auto& info_px_pair : info_px_pairs)
    addPixelToPC(grid, pc, info_px_pair.px, r, g, b);
}

//
// coordination callbacks
//

void mapCB(const grid_mapping::OccupancyGridConstPtr& msg)
{
  received_first_map = true;

  try {
    geometry_msgs::TransformStamped tfs;
    string my_frame = tf_prefix + "/map";
    tfs = tfBuffer.lookupTransform(my_frame, msg->header.frame_id, ros::Time(0));

    grid_mapping::Point trans(tfs.transform.translation.x, tfs.transform.translation.y);
    grid_mapping::OccupancyGrid new_map = *msg;
    new_map.origin.x += trans.x;
    new_map.origin.y += trans.y;
    grid_mapping::OccupancyGridConstPtr new_map_ptr(new grid_mapping::OccupancyGrid(new_map));

    ang_grid.insertMap(new_map_ptr);
    viz_map_pub.publish(ang_grid.createROSOGMsg());
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s failed to insert map with frame_id %s:\n%s", tf_prefix.c_str(),
        msg->header.frame_id.c_str(), ex.what());
  }
}

void goalPoseCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
  grid_mapping::Point pt(msg->pose.position.x, msg->pose.position.y);
  pt.y += robot1_y_offset;

  GoalIDPair goal_pair;
  goal_pair.point = pt;
  goal_pair.id = msg->header.seq;
  goals.push_back(goal_pair);
}

void panPoseCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
  grid_mapping::Point pt(msg->pose.position.x, msg->pose.position.y);
  pt.y += robot1_y_offset;
  pan_locations.push_back(pt);

  int id = msg->header.seq;
  if (goals.size() > 0) {
    for (int i = 0; i < goals.size(); ++i) {
      if (goals[i].id == id) {
        goals.erase(goals.begin()+i);
        ROS_INFO("robot%d matched goal and panorama with id: %d", robot_id, id);
        return;
      }
    }
    ROS_INFO("robot%d found no matching goal for panorama with id: %d", robot_id, id);
    std::stringstream ss;
    for (int i = 0; i < goals.size()-1; ++i) {
      ss << goals[i].id << ", ";
    }
    ss << goals.back().id;
    string id_str = ss.str();
    ROS_INFO("current list of ids: %s", id_str.c_str());
  }
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
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Publisher vel_pub, pan_grid_pub, skel_pub, pan_pose_pub, goal_pose_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("velocity_commands", 10);
  viz_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_2D", 2);
  pan_grid_pub = nh.advertise<grid_mapping::OccupancyGrid>("angle_grid", 2);
  skel_pub = nh.advertise<PointCloud2>("skeleton", 2);
  pan_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pan_pose", 2);
  goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 2);

  int number_of_robots;
  double scan_range_min, scan_range_max;
  if (!pnh.getParam("tf_prefix", tf_prefix) ||
      !pnh.getParam("robot_id", robot_id) ||
      !nh.getParam("/number_of_robots", number_of_robots) ||
      !nh.getParam("/scan_range_min", scan_range_min) ||
      !nh.getParam("/scan_range_max", scan_range_max)) {
    ROS_FATAL("main(...): failed to read params from server");
    exit(EXIT_FAILURE);
  }

  if (robot_id == 1)
    received_first_map = true; 

  // initialize angle grid for planning
  ang_grid.range_min = scan_range_min;
  ang_grid.range_max = scan_range_max;

  vector<ros::Subscriber> coord_subs;
  for (int i = 1; i <= number_of_robots; ++i) {
    if (i == robot_id)
      continue;

    string robot_ns = "/robot" + std::to_string(i);
    coord_subs.push_back(nh.subscribe(robot_ns + "/angle_grid", 2, &mapCB));
    coord_subs.push_back(nh.subscribe(robot_ns + "/goal_pose", 2, &goalPoseCB));
    coord_subs.push_back(nh.subscribe(robot_ns + "/pan_pose", 2, &panPoseCB));
  }

  /*
   * Initialize action servers for panorama capture and navigation
   */

  string pan_server_name = "/" + tf_prefix + "/panorama_action_server";
  PanAC pan_ac(pan_server_name.c_str(), true);
  ROS_INFO("Waiting for action server to start: %s", pan_server_name.c_str());
  pan_ac.waitForServer();
  ROS_INFO("%s is ready", pan_server_name.c_str());

  string nav_server_name = "/" + tf_prefix + "/move_base";
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
   * exploration loop
   */

  int pan_count = 0;
  while (ros::ok()) {

    /*
     * If not robot1, wait for robot1 to complete first panorama before starting
     * to expore
     */

    if (!received_first_map) {
      ros::spinOnce();
      countdown.sleep();
      ROS_INFO("robot%d is waiting for first map from robot1", robot_id);
    }

    /*
     * Collect panorama, insert it into the angle grid, and publish the angle
     * grid and 2D rviz version
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
    AngleGrid pan_grid(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1);
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

    // get the latest information from the team
    ros::spinOnce();

    // read panoaram capture location
    rosbag::Bag panbag;
    panbag.open(pan_file, rosbag::bagmode::Read);
    geometry_msgs::TransformStamped pan_pose;
    for (auto m : rosbag::View(panbag, rosbag::TopicQuery("panorama_pose"))) {
      auto msg = m.instantiate<geometry_msgs::TransformStamped>();
      if (msg) {
        geometry_msgs::PoseStamped pan_pose;
        pan_pose.header = msg->header;
        pan_pose.header.seq = robot_id*10 + pan_count;
        pan_pose.pose.position.x = msg->transform.translation.x;
        pan_pose.pose.position.y = msg->transform.translation.y;
        pan_pose.pose.position.z = 0.0;
        pan_pose.pose.orientation = msg->transform.rotation;
        pan_pose_pub.publish(pan_pose);
        break;
      }
    }
    grid_mapping::Point panorama_position(pan_pose.transform.translation.x,
        pan_pose.transform.translation.y);
    pan_locations.push_back(panorama_position);

    // compute skeleton
    cv::Mat skel;
    computeSkeleton(ang_grid, skel, 2);

    // partition skel
    vector<cv::Point> pan_pixels;
    for (auto pt : pan_locations) {
      cv::Point po;
      ang_grid.positionToSubscripts(panorama_position, po.y, po.x);
      pan_pixels.push_back(po);
    }
    regions_vec regions = partitionSkeleton(skel, pan_pixels);

    // find the point of each region with the hightest CSQMI
    CSQMI objective(DepthCamera(180, scan_range_max, scan_range_min), 0.03);
    vector<InfoPxPair> goal_pairs;
    for (auto& reg : regions) {
      vector<InfoPxPair> reg_csqmi = objective.csqmi(ang_grid, reg);
      goal_pairs.push_back(*max_element(reg_csqmi.begin(), reg_csqmi.end(),
            InfoPxPair::compare));
    }

    // sort highest to lowest csqmi
    std::sort(goal_pairs.begin(), goal_pairs.end(), InfoPxPair::compare);
    std::reverse(goal_pairs.begin(), goal_pairs.end());

    // get the latest information from the team
    ros::spinOnce();

    // choose the goal point
    grid_mapping::Point goal_pt;
    auto goal_it = goal_pairs.begin();
    bool goal_found = false;
    while (!goal_found && goal_it != goal_pairs.end()) {
      goal_pt = ang_grid.subscriptsToPosition(goal_it->px.y, goal_it->px.x);

      // make sure agent goals do not collide
      for (auto goal_pair : goals) {
        if ((goal_pt - goal_pair.point).norm() < 0.5) {
          ++goal_it;
          continue;
        }
      }
      goal_found = true;
    }

    // if no goal was found, wait for a new map and then start planning again
    if (!goal_found) {
      ROS_WARN("failed to find a goal... will wait for new map");
      received_first_map = false;
      continue;
    }

    /*
     * Publish PointCloud2 of the skeleton, max points, and goal
     */

    CloudXYZRGB skel_pc = skeletonToPC(ang_grid, regions, 0, 0, 255);
    addPixelsToPC(ang_grid, skel_pc, pan_pixels, 255, 0, 0);
    addPixelsToPC(ang_grid, skel_pc, goal_pairs, 255, 255, 0);
    addPixelToPC(ang_grid, skel_pc, goal_it->px, 0, 255, 0);

    PointCloud2 skel_pc_msg;
    pcl::toROSMsg(skel_pc, skel_pc_msg);
    skel_pc_msg.header.frame_id = "/" + tf_prefix + "/map";
    skel_pub.publish(skel_pc_msg);

    /*
     * Navigate to the goal point
     */

    move_base_msgs::MoveBaseGoal action_goal;
    action_goal.target_pose.pose.position.x = goal_pt.x;
    action_goal.target_pose.pose.position.y = goal_pt.y;
    action_goal.target_pose.header.stamp = ros::Time::now();
    action_goal.target_pose.header.frame_id = tf_prefix + "/map";
    action_goal.target_pose.pose.orientation.w = 1.0;
    action_goal.target_pose.header.seq = robot_id*10 + pan_count + 1;
    goal_pose_pub.publish(action_goal.target_pose);

    move_ac.sendGoal(action_goal, &moveDoneCB, &moveActiveCB, &moveFeedbackCB);
    move_ac.waitForResult();

    ROS_INFO("Completed iteration loop %d", pan_count);

    // get the latest information from the team
    ros::spinOnce();
  }

  return 0;
}
