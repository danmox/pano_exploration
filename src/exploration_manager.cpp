#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_mapping/OccupancyGrid.h>
#include <csqmi_exploration/PanGoal.h>

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

int robot_id;
int pan_count = 0;
volatile int shared_goals_count = 0;
volatile bool received_new_map = false, move_base_succeeded = true;
string tf_prefix;
string pan_file;
AngleGrid ang_grid(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1);
ros::Publisher viz_map_pub, pan_grid_pub, pan_pose_pub;
vector<grid_mapping::Point> pan_locations;
vector<GoalIDPair> goals;
tf2_ros::Buffer tfBuffer;
double scan_range_min, scan_range_max;
std::shared_ptr<PanAC> pan_ac;
std::shared_ptr<MoveAC> move_ac;

//
// panorama action functions
//

void panFeedbackCB(const panorama::PanoramaFeedbackConstPtr& feedback)
{
  ROS_INFO("panFeedbackCB(...): Captured frame %d of 72",
      feedback->frames_captured);
}

void panActiveCB()
{
  ROS_INFO("panActiveCB(): Capturing panorama...");
}

void panDoneCB(const actionlib::SimpleClientGoalState& state,
    const panorama::PanoramaResultConstPtr& result)
{
  ROS_INFO("panDoneCB(...): Panorama completed with %s and saved to %s",
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
  ROS_INFO("moveActionCB(): Navigating to goal...");
}

void moveDoneCB(const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("moveDoneCB(...): Navigation completed with status: %s",
      state.toString().c_str());
  if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
    move_base_succeeded = false;
  }
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

bool fetchTransform(string source_frame, geometry_msgs::TransformStamped& trans)
{
  bool res = false;
  try {
    string my_frame = tf_prefix + "/map";
    trans = tfBuffer.lookupTransform(my_frame, source_frame, ros::Time(0));
    res = true;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("fetchTransform(...): %s failed to fetch transform:\n%s",
        tf_prefix.c_str(), ex.what());
  }
  return res;
}

void mapCB(const grid_mapping::OccupancyGridConstPtr& msg)
{
  static vector<grid_mapping::OccupancyGrid> msgs_to_process;
  received_new_map = true;
  msgs_to_process.push_back(*msg);

  for (auto& grid : msgs_to_process) {
    geometry_msgs::TransformStamped tfs;
    if (fetchTransform(grid.header.frame_id, tfs)) {
      grid_mapping::OccupancyGridConstPtr cpt(new grid_mapping::OccupancyGrid(grid));
      ang_grid.insertMap(msg, tfs);
      ROS_INFO("mapCB(...): %s inserted map with frame_id %s", tf_prefix.c_str(),
          msg->header.frame_id.c_str());
      viz_map_pub.publish(ang_grid.createROSOGMsg());
    } else {
      ROS_WARN("mapCB(...): %s failed to insert map with frame_id %s",
          tf_prefix.c_str(), msg->header.frame_id.c_str());
      return;
    }
  }
  ROS_INFO("mapCB(...): processed %d messages in msgs_to_process",
      (int)msgs_to_process.size());
  msgs_to_process.clear();
}

void goalPoseCB(const csqmi_exploration::PanGoalConstPtr& msg)
{
  static vector<csqmi_exploration::PanGoal> msgs_to_process;
  ++shared_goals_count;
  msgs_to_process.push_back(*msg);

  for (csqmi_exploration::PanGoal& goal_msg : msgs_to_process) {
    geometry_msgs::TransformStamped tfs;
    if (fetchTransform(goal_msg.frame_id, tfs)) {
      grid_mapping::Point in_pt(goal_msg.x, goal_msg.y);
      grid_mapping::Point pt = grid_mapping::Point::transformPoint(tfs, in_pt);

      GoalIDPair goal_pair;
      goal_pair.point = pt;
      goal_pair.id = goal_msg.goal_id;
      goals.push_back(goal_pair);
      ROS_INFO("goalPoseCB(...): %s added goal pose with id %d to goals",
          tf_prefix.c_str(), goal_pair.id);
    } else {
      ROS_WARN("goalPoseCB(...): %s failed to add goal pose with ID %d to goals",
          tf_prefix.c_str(), goal_msg.goal_id);
      return;
    }
  }
  ROS_INFO("goalPoseCB(...): processed %d messages in msgs_to_process",
      (int)msgs_to_process.size());
  msgs_to_process.clear();
}

void panPoseCB(const csqmi_exploration::PanGoalConstPtr& msg)
{
  static vector<csqmi_exploration::PanGoal> msgs_to_process;
  msgs_to_process.push_back(*msg);

  for (csqmi_exploration::PanGoal pan_msg : msgs_to_process) {
    int id = pan_msg.goal_id;
    geometry_msgs::TransformStamped tfs;
    if (fetchTransform(pan_msg.frame_id, tfs)) {
      grid_mapping::Point in_pt(pan_msg.x, pan_msg.y);
      grid_mapping::Point pt = grid_mapping::Point::transformPoint(tfs, in_pt);
      pan_locations.push_back(pt);
      ROS_INFO("panPoseCB(...): %s added panorama pose with id %d to "
          "pan_locations", tf_prefix.c_str(), id);

      if (goals.size() == 0)
        return;

      for (int i = 0; i < goals.size(); ++i) {
        if (goals[i].id == id) {
          goals.erase(goals.begin()+i);
          ROS_INFO("panPoseCB(...): %s found goal and panorama with matching id: "
              "%d", tf_prefix.c_str(), id);
          return;
        }
      }

      ROS_INFO("panPoseCB(...): %s found no matching goal for panorama with id: "
          "%d", tf_prefix.c_str(), id);
      std::stringstream ss;
      for (int i = 0; i < goals.size()-1; ++i) { ss << goals[i].id << ", "; }
      ss << goals.back().id;
      string id_str = ss.str();
      ROS_INFO("panPoseCB(...): current list of goal ids is: %s", id_str.c_str());
    } else {
      ROS_INFO("panPoseCB(...): %s failed to add panorama with ID %d to captured "
          "panorama list", tf_prefix.c_str(), pan_msg.goal_id);
    }
  }
  ROS_INFO("panPoseCB(...): processed %d messages in msgs_to_process",
      (int)msgs_to_process.size());
  msgs_to_process.clear();
}

//
// exploration functions
//

/*
 * Collect panorama, insert it into the angle grid, and publish the angle
 * grid, 2D rviz version, and panorama capture location
 */

bool capturePanorama()
{
  panorama::PanoramaGoal pan_goal;
  pan_goal.file_name = tf_prefix + "pan" + std::to_string(++pan_count);
  pan_ac->sendGoal(pan_goal, &panDoneCB, &panActiveCB, &panFeedbackCB);
  pan_ac->waitForResult();
  if (pan_file.size() == 0) {
    ROS_WARN("capturePanorama(): Panorama action returned no file");
    --pan_count;
    return false;
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

  // read panoaram capture location
  rosbag::Bag panbag;
  panbag.open(pan_file, rosbag::bagmode::Read);
  csqmi_exploration::PanGoal pan_pose_msg;
  for (auto m : rosbag::View(panbag, rosbag::TopicQuery("panorama_pose"))) {
    auto msg = m.instantiate<geometry_msgs::TransformStamped>();
    if (msg) {
      pan_pose_msg.frame_id = msg->header.frame_id;
      pan_pose_msg.goal_id = robot_id*100 + pan_count;
      pan_pose_msg.x = msg->transform.translation.x;
      pan_pose_msg.y = msg->transform.translation.y;
      pan_pose_pub.publish(pan_pose_msg);
      ROS_INFO("capturePanorama(): %s published panorama pose with id %d",
          tf_prefix.c_str(), pan_pose_msg.goal_id);
      break;
    }
  }
  panbag.close();
  grid_mapping::Point panorama_position;
  panorama_position.x = pan_pose_msg.x;
  panorama_position.y = pan_pose_msg.y;
  pan_locations.push_back(panorama_position);

  return true;
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

  ros::Publisher vel_pub, skel_pub, goal_pose_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("velocity_commands", 10);
  viz_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_2D", 2);
  pan_grid_pub = nh.advertise<grid_mapping::OccupancyGrid>("angle_grid", 2);
  skel_pub = nh.advertise<PointCloud2>("skeleton", 2);
  pan_pose_pub = nh.advertise<csqmi_exploration::PanGoal>("pan_pose", 2);
  goal_pose_pub = nh.advertise<csqmi_exploration::PanGoal>("goal_pose", 2);

  int number_of_robots;
  bool leader;
  if (!pnh.getParam("tf_prefix", tf_prefix) ||
      !pnh.getParam("robot_id", robot_id) ||
      !pnh.getParam("leader", leader) ||
      !nh.getParam("/number_of_robots", number_of_robots) ||
      !nh.getParam("/scan_range_min", scan_range_min) ||
      !nh.getParam("/scan_range_max", scan_range_max)) {
    ROS_FATAL("main(...): failed to read params from server");
    exit(EXIT_FAILURE);
  }
  ang_grid.frame_id = tf_prefix + "/map";

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
  pan_ac.reset(new PanAC(pan_server_name.c_str(), true));
  ROS_INFO("main(...): Waiting for action server to start: %s",
      pan_server_name.c_str());
  pan_ac->waitForServer();
  ROS_INFO("main(...): %s is ready", pan_server_name.c_str());

  string nav_server_name = "/" + tf_prefix + "/move_base";
  move_ac.reset(new MoveAC(nav_server_name.c_str(), true));
  ROS_INFO("main(...): Waiting for action server to start: %s",
      nav_server_name.c_str());
  move_ac->waitForServer();
  ROS_INFO("main(...): %s is ready", nav_server_name.c_str());

  /*
   * Give the other components/robots time to load before staring exploration
   */

  ros::Rate countdown(1);
  for (int i = 5; i > 0; --i) {
    ROS_INFO("main(...): Beginning exploration in %d seconds...", i);
    countdown.sleep();
  }

  /*
   * if this robot is a leader, capture an initial panorama before beginning the
   * exploration loop; otherwise, wait for goals in robot_id order to abate a
   * race condition between the non leader members of the team
   */

  if (leader) {
    capturePanorama();
  } else {
    while (shared_goals_count < robot_id-1) {
      countdown.sleep(); countdown.sleep();
      ROS_INFO("main(...): robot%d is waiting for %d goals to be received "
          "before beginning exploration", robot_id, robot_id-1);
      ros::spinOnce();
    }
  }

  /*
   * exploration loop
   */

  while (ros::ok()) {

    // get the latest information from the team
    ros::spinOnce();

    /*
     * Compute next panorama capture location
     */

    // compute skeleton
    cv::Mat skel;
    computeSkeleton(ang_grid, skel, 2);

    // get the latest information from the team
    ros::spinOnce();

    // partition skel
    vector<cv::Point> pan_pixels;
    for (auto pt : pan_locations) {
      cv::Point po;
      ang_grid.positionToSubscripts(pt, po.y, po.x);
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

      if (goals.size() == 0) {
        goal_found = true;
        break;
      }

      // make sure agent goals do not collide
      for (auto goal_pair : goals) {
        double goal_diff = (goal_pt - goal_pair.point).norm();
        if (goal_diff < 0.5) {
          ROS_INFO_STREAM("main(...): goal: " << goal_pt << " in proximity to "
              "goal: " << goal_pair.point << " with id: " << goal_pair.id);
          ++goal_it;
          continue;
        } else {
          goal_found = true;
          break;
        }
      }
    }

    // if no goal was found, wait for a new map and then start planning again
    if (!goal_found) {
      ROS_WARN("main(...): failed to find a goal... will wait for new map");
      received_new_map = false;
      while (!received_new_map) {
        countdown.sleep();
        ROS_INFO("main(...): robot%d is waiting for a new map", robot_id);
        ros::spinOnce();
      }
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
    action_goal.target_pose.header.stamp = ros::Time::now();
    action_goal.target_pose.header.frame_id = tf_prefix + "/map";
    action_goal.target_pose.pose.position.x = goal_pt.x;
    action_goal.target_pose.pose.position.y = goal_pt.y;
    action_goal.target_pose.pose.orientation.w = 1.0;

    csqmi_exploration::PanGoal goal_pose_msg;
    goal_pose_msg.frame_id = tf_prefix + "/map";
    goal_pose_msg.x = goal_pt.x;
    goal_pose_msg.y = goal_pt.y;
    goal_pose_msg.goal_id = robot_id*100 + pan_count + 1;
    goal_pose_pub.publish(goal_pose_msg);

    move_ac->sendGoal(action_goal, &moveDoneCB, &moveActiveCB, &moveFeedbackCB);
    move_ac->waitForResult();
    if (!move_base_succeeded) {
      ROS_INFO("main(...): Navigation failed. Restarting planning process.");
      continue;
    }

    /*
     * Collect panorama, insert it into the angle grid, and publish the angle
     * grid and 2D rviz version
     */

    capturePanorama();

    ROS_INFO("main(...): Completed iteration loop %d", pan_count);
  }

  return 0;
}
