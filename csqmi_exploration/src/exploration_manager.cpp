#include "csqmi_exploration/exploration_manager.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_mapping/OccupancyGrid.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

#include <sstream>
#include <algorithm>
#include <functional>
#include <stack>

namespace csqmi_exploration {

typedef std::vector<std::vector<cv::Point>> regions_vec;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

ExplorationManager::ExplorationManager(ros::NodeHandle pnh_, ros::NodeHandle nh_) :
  nh(nh_),
  pnh(pnh_),
  pan_count(0),
  heartbeat(0),
  shared_goals_count(0),
  received_new_map(false),
  navigation_succeeded(false),
  navigation_in_progress(false),
  ang_grid(grid_mapping::Point(0.0, 0.0), 0.1, 1, 1)
{

  //
  // initialize publishers / subscribers / parameters
  //

  tfListener.reset(new tf2_ros::TransformListener(tfBuffer));

  activate_sub = nh.subscribe("activate", 2, &ExplorationManager::activateCB, this);
  viz_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_2D", 2);
  pan_grid_pub = nh.advertise<grid_mapping::OccupancyGrid>("angle_grid", 2);
  skel_pub = nh.advertise<sensor_msgs::PointCloud2>("skeleton", 2);
  goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 2);
  frontier_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("frontier_map", 2);

  if (!pnh.getParam("tf_prefix", tf_prefix) ||
      !pnh.getParam("robot_id", robot_id) ||
      !pnh.getParam("leader", leader) ||
      !pnh.getParam("pan_server_name", pan_server_name) ||
      !pnh.getParam("nav_server_name", nav_server_name) ||
      !pnh.getParam("world_frame", world_frame) ||
      !pnh.getParam("robot_frame", robot_frame)) {
    ROS_FATAL("[ExplorationManager]: failed to read private params from server");
    exit(EXIT_FAILURE);
  }
  if (!nh.getParam("/number_of_robots", number_of_robots) ||
      !nh.getParam("/scan_range_min", scan_range_min) ||
      !nh.getParam("/scan_range_max", scan_range_max) ||
      !nh.getParam("/grid_resolution", grid_res) ||
      !nh.getParam("/pan_goal_tol", pan_goal_tol) ||
      !nh.getParam("/goal_method", goal_method)) {
    ROS_FATAL("[ExplorationManager]: failed to read global params from server");
    exit(EXIT_FAILURE);
  }
  ang_grid = grid_mapping::AngleGrid(grid_mapping::Point(0.0, 0.0), grid_res, 1, 1);

  // CSQMI objective function
  objective.reset(new CSQMI(DepthCamera(180, scan_range_max, scan_range_min), 0.03));

  // TODO: get other robot ids as array parameter
  // TODO: change "robot" to "scarab"
  if (number_of_robots > 1) {
    for (int i = 1; i <= number_of_robots; ++i) {
      if (i == robot_id)
        continue;

      std::string robot_ns = "/robot" + std::to_string(i);
      coord_subs.push_back(nh.subscribe(robot_ns + "/angle_grid", 2, &ExplorationManager::mapCB, this));
      coord_subs.push_back(nh.subscribe(robot_ns + "/goal_pose", 2, &ExplorationManager::goalPoseCB, this));
    }
  }

  //
  // initialize angle grid for planning
  //

  ang_grid.frame_id = world_frame;
  ang_grid.range_min = scan_range_min;
  ang_grid.range_max = scan_range_max;

  //
  // Initialize action servers for panorama capture and navigation
  //

  pan_ac.reset(new PanAC(pan_server_name.c_str(), true));
  ROS_INFO("[ExplorationManager]: Waiting for action server to start: %s", pan_server_name.c_str());
  pan_ac->waitForServer();
  ROS_INFO("[ExplorationManager]: %s is ready", pan_server_name.c_str());

  move_ac.reset(new MoveAC(nav_server_name.c_str(), true));
  ROS_INFO("[ExplorationManager]: Waiting for action server to start: %s", nav_server_name.c_str());
  move_ac->waitForServer();
  ROS_INFO("[ExplorationManager]: %s is ready", nav_server_name.c_str());

}

//
// panorama action callback functions
//

void panFeedbackCB(const panorama::PanoramaFeedbackConstPtr& feedback)
{
  //ROS_INFO("[ExplorationManager]: frames captured: %d",
  //    feedback->frames_captured);
}

void panActiveCB()
{
  ROS_INFO("[ExplorationManager]: Capturing panorama...");
}

// when a panorama is captured, the action server returns the absolute file path
// to the panorama bag file
void ExplorationManager::panDoneCB(const actionlib::SimpleClientGoalState& state,
                                   const panorama::PanoramaResultConstPtr& result)
{
  ROS_INFO("[ExplorationManager]: Panorama completed with %s and saved to %s", state.toString().c_str(), result->full_file_name.c_str());
  if (state.toString().compare("SUCCEEDED") == 0) {
    pan_file = result->full_file_name;
  }
}

// allow control over exploration
void ExplorationManager::activateCB(const std_msgs::Bool::ConstPtr& msg)
{
  heartbeat = ros::Time::now();
}

//
// hfn action action callbacks
//

void moveFeedbackCB(const scarab_msgs::MoveFeedbackConstPtr& msg)
{
}

void moveActiveCB()
{
  ROS_INFO("[ExplorationManager]: Navigating to goal...");
}

void ExplorationManager::moveDoneCB(const actionlib::SimpleClientGoalState& state,
                                    const scarab_msgs::MoveResultConstPtr& result)
{
  navigation_in_progress = false;
  ROS_INFO("[ExplorationManager]: Navigation completed with status: %s", state.toString().c_str());
  if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    navigation_succeeded = false;
  else
    navigation_succeeded = true;
}

//
// visualization functions: for displaying the skeleton used for computing CSQMI
// as a pointcloud in rviz
//

pcl::PointXYZRGB pixelToPCLPoint(const grid_mapping::AngleGrid& grid,
                                 cv::Point pixel,
                                 int r, int g, int b)
{
  grid_mapping::Point pt = grid.subscriptsToPosition(pixel.y, pixel.x);
  pcl::PointXYZRGB pcl_point(r,g,b);
  pcl_point.x = pt.x;
  pcl_point.y = pt.y;
  return pcl_point;
}

CloudXYZRGB skeletonToPC(const grid_mapping::AngleGrid& grid,
                         const regions_vec& regions,
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

void addPixelToPC(const grid_mapping::AngleGrid& grid,
                  CloudXYZRGB& point_cloud,
                  cv::Point pixel,
                  int r, int g, int b)
{
  grid_mapping::Point point = grid.subscriptsToPosition(pixel.y, pixel.x);
  pcl::PointXYZRGB pcl_point(r,g,b);
  pcl_point.x = point.x;
  pcl_point.y = point.y;
  point_cloud.points.push_back(pcl_point);
  point_cloud.width++;
}

void addPointToPC(CloudXYZRGB& point_cloud,
                  grid_mapping::Point point,
                  int r, int g, int b)
{
  pcl::PointXYZRGB pcl_point(r,g,b);
  pcl_point.x = point.x;
  pcl_point.y = point.y;
  point_cloud.points.push_back(pcl_point);
  point_cloud.width++;
}

void addPointsToPC(CloudXYZRGB& point_cloud,
                   std::vector<grid_mapping::Point>& points,
                   int r, int g, int b)
{
  for (auto pt : points)
    addPointToPC(point_cloud, pt, r, g, b);
}

void addPixelsToPC(const grid_mapping::AngleGrid& grid,
                   CloudXYZRGB& point_cloud,
                   std::vector<cv::Point>& pixels,
                   int r, int g, int b)
{
  for (auto& pixel : pixels) {
    addPixelToPC(grid, point_cloud, pixel, r, g, b);
  }
}

void addPixelsToPC(const grid_mapping::AngleGrid& grid,
                   CloudXYZRGB& point_cloud,
                   std::vector<InfoPxPair>& info_px_pairs,
                   int r, int g, int b)
{
  for (auto& info_px_pair : info_px_pairs) {
    addPixelToPC(grid, point_cloud, info_px_pair.px, r, g, b);
  }
}

//
// coordination callbacks
//
// Agents exchange goals they are going to, goals they have completed, and map
// updates after capturing panoramas. Goals a list of goals other agents are
// actively pursuing is maintained and candidate goals are checked against this
// list to prevent robots choosing the same location at which to capture a pano.
// It is assumed that all agents are operating in a shared global reference
// frame.
//

bool ExplorationManager::getWorldTrans(const std::string source_frame,
                                       geometry_msgs::TransformStamped& trans) const
{
  try {
    trans = tfBuffer.lookupTransform(world_frame, source_frame, ros::Time(0));
    return true;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("[ExplorationManager]: %s failed to fetch transform:\n%s", tf_prefix.c_str(), ex.what());
  }
  return false;
}

void ExplorationManager::mapCB(const grid_mapping::OccupancyGridConstPtr& msg)
{
  received_new_map = true;
  ang_grid.insertMap(msg);
  ROS_INFO("[ExplorationManager]: %s inserted map with frame_id %s", tf_prefix.c_str(), msg->header.frame_id.c_str());
  viz_map_pub.publish(ang_grid.createROSOGMsg());
  grid_mapping::Point pan_pt(msg->origin.x, msg->origin.y);
  pan_locations.push_back(pan_pt);

  // TODO remove completed pan from goal_locations
}

void ExplorationManager::goalPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ++shared_goals_count; // TODO base this off of index in shared robot_id array
  grid_mapping::Point goal_pt(msg->pose.position.x, msg->pose.position.y);
  goal_locations.push_back(goal_pt);
}

//
// exploration functions
//

// Capture panorama, insert it into the angle grid, and publish the angle grid,
// flattened 2D grid for rviz, and panorama capture location
bool ExplorationManager::capturePanorama()
{
  // get current pose of the robot
  geometry_msgs::TransformStamped tfs;
  getWorldTrans(robot_frame, tfs);
  grid_mapping::Point pose(tfs.transform.translation.x, tfs.transform.translation.y);

  // call panorama action
  panorama::PanoramaGoal pan_goal;
  pan_goal.file_name = tf_prefix + "pan" + std::to_string(++pan_count);
  pan_ac->sendGoal(pan_goal, std::bind(&ExplorationManager::panDoneCB, this, std::placeholders::_1, std::placeholders::_2), &panActiveCB, &panFeedbackCB);
  ROS_INFO("[ExplorationManager]: sent panorama goal to action server");
  pan_ac->waitForResult();
  if (pan_file.size() == 0) {
    ROS_ERROR("[ExplorationManager]: Panorama action returned no file");
    --pan_count;
    return false;
  }

  // store the capture location
  pan_locations.push_back(pose);

  // create map from the panorama
  grid_mapping::AngleGrid pan_grid(pose, grid_res, 1, 1);
  pan_grid.range_min = scan_range_min;
  pan_grid.range_max = scan_range_max;
  pan_grid.insertPanorama(pan_file);

  // often other robots are captured in a panorama and marked as obstacles in
  // the resulting map; however, their location should be marked as free
  for (int i = 1; i <= number_of_robots; ++i) {
    if (i == robot_id) { // TODO change robot_id method
      continue;
    }

    std::string src_frame = "robot" + std::to_string(i) + "/base_link";
    geometry_msgs::TransformStamped other_tfs;
    if (getWorldTrans(src_frame, other_tfs)) {

      // recover the pose of this robot and other robot in a common frame
      grid_mapping::Point other_pose(other_tfs.transform.translation.x, other_tfs.transform.translation.y);

      // if the distance between poses is within the maximum scan range then it
      // is possible the other robot was captured: mark its pose as free
      if ((other_pose - pose).norm() < scan_range_max + 0.5) {
        ROS_INFO_STREAM("[ExplorationManager] Removing robot" << i << " at " << other_pose << " from my map");
        pan_grid.updateRobotCells(other_pose, 0.5);
      }
    }
  }

  // create grid_mapping::OccupancyGrid ROS message of the panorama map and
  // share it with the other robots
  auto pan_grid_msg = pan_grid.createROSMsg();
  pan_grid_pub.publish(pan_grid_msg);

  // insert the panorama map into this robot's map
  ang_grid.insertMap(pan_grid_msg);

  // publish a 2D visualization of this robot's angle grid
  viz_map_pub.publish(ang_grid.createROSOGMsg());

  return true;
}

// returns true if pt is within tol of any of pts
bool inProximity(const grid_mapping::Point& pt,
                 const std::vector<grid_mapping::Point>& pts,
                 const double tol)
{
  for (const grid_mapping::Point& point : pts)
    if ((pt - point).norm() < tol)
      return true;
  return false;
}


// find next panorama location using csqmi
bool ExplorationManager::csqmiGoal(grid_mapping::Point& goal_pt)
{
     // compute skeleton
    cv::Mat skel;
    computeSkeleton(ang_grid, skel, 2);

    // get the latest information from the team
    ros::spinOnce();

    // partition skeleton into regions divided by past panorama capture locations
    std::vector<cv::Point> pan_pixels;
    for (auto pt : pan_locations) {
      cv::Point po;
      ang_grid.positionToSubscripts(pt, po.y, po.x);
      pan_pixels.push_back(po);
    }
    regions_vec regions = partitionSkeleton(skel, pan_pixels);

    // find the point of each region with the hightest CSQMI: these are the goals
    std::vector<InfoPxPair> goal_pairs;
    for (auto& indices : regions) {
      std::vector<InfoPxPair> reg = objective->csqmi(ang_grid, indices);
      auto max_el = *max_element(reg.begin(), reg.end(), InfoPxPair::lesser);
      goal_pairs.push_back(max_el);
    }

    // sort highest to lowest csqmi
    std::sort(goal_pairs.begin(), goal_pairs.end(), InfoPxPair::greater);

    // get the latest information from the team
    ros::spinOnce();

    // choose the goal point, ensuring agents don't choose the same point
    bool goal_found = false;
    auto it = goal_pairs.begin();
    for (; it != goal_pairs.end(); ++it) {
      goal_pt = ang_grid.subscriptsToPosition(it->px.y, it->px.x);

      // make sure agent goals are not in proximity
      if (!inProximity(goal_pt, goal_locations, pan_goal_tol)) {
        goal_found = true;
        break;
      } else {
        ROS_INFO_STREAM("[ExplorationManager]: ignoring goal " << goal_pt <<
                        "in proximity to other goal");
      }
    }

    if (!goal_found)
      return false;

    // Publish sensor_msgs::PointCloud2 of the skeleton, max points, and goal

    CloudXYZRGB skel_pc = skeletonToPC(ang_grid, regions, 0, 0, 255);
    addPixelsToPC(ang_grid, skel_pc, pan_pixels, 255, 0, 0);
    addPixelsToPC(ang_grid, skel_pc, goal_pairs, 255, 255, 0);
    addPixelToPC(ang_grid, skel_pc, it->px, 0, 255, 0);

    sensor_msgs::PointCloud2 skel_pc_msg;
    pcl::toROSMsg(skel_pc, skel_pc_msg);
    skel_pc_msg.header.frame_id = world_frame;
    skel_pub.publish(skel_pc_msg);

    return true;
}


// find next panorama location using near frontier
bool ExplorationManager::frontierGoal(grid_mapping::Point& goal_pt)
{
  std::vector<std::vector<grid_mapping::Point>> frontiers = findFrontiers();
  if (frontiers.empty()) {
    ROS_WARN("[ExplorationManager] no frontiers found");
    return false;
  }

  // find near frontier
  double min_frontier_dist = -1.0;
  for (auto frontier : frontiers) {

    // compute centroid
    double centroid_x = 0.0, centroid_y = 0.0;
    for (grid_mapping::Point pt : frontier) {
      centroid_x += pt.x;
      centroid_y += pt.y;
    }
    centroid_x /= (double)frontier.size();
    centroid_y /= (double)frontier.size();
    int centroid = ang_grid.positionToIndex(centroid_x, centroid_y);

    // if the centroid is not in free space, attempt to find some nearby
    bool centroid_found = true;
    if (ang_grid.cellProb(centroid) > 0.05) {
      ROS_INFO("[ExplorationManager] relocating frontier centroid");
      centroid_found = false;
      for (int cell : ang_grid.neighborIndices(centroid, 0.5)) {
        if (ang_grid.cellProb(centroid) > 0.05) {
          centroid = cell;
          centroid_found = true;
        }
      }
    }

    if (!centroid_found) {
      ROS_ERROR("[ExplorationManager] no free space near computed frontier centroid");
      continue;
    }

    // compute path distance and compare to other frontiers

    grid_mapping::Point robot_pt = pan_locations.back(); // TODO for multi-agent
    grid_mapping::Point frontier_pt = ang_grid.indexToPosition(centroid);
    double new_dist = (frontier_pt - robot_pt).norm();
    if (min_frontier_dist < 0.0 || new_dist < min_frontier_dist) {
      goal_pt = frontier_pt;
      min_frontier_dist = new_dist;
    }
  }

  if (min_frontier_dist < 0.0) {
    ROS_ERROR("[ExplorationManager] no nearest frontier found");
    return false;
  }

  //
  // Publish frontiers as point cloud
  //

  CloudXYZRGB frontiers_pc;
  for (auto frontier : frontiers)
    addPointsToPC(frontiers_pc, frontier, 0, 0, 255);
  addPointToPC(frontiers_pc, goal_pt, 0, 255, 0);
  frontiers_pc.width = frontiers_pc.points.size();
  frontiers_pc.height = 1;
  frontiers_pc.is_dense = false;

  sensor_msgs::PointCloud2 ros_pc_msg;
  pcl::toROSMsg(frontiers_pc, ros_pc_msg);
  ros_pc_msg.header.frame_id = world_frame;
  skel_pub.publish(ros_pc_msg);

  return true;
}

//
// main exploration loop
//

void ExplorationManager::explorationLoop()
{
  ROS_INFO("[ExplorationManager]: beginning exploration loop");

  //
  // Give the other components/robots time to load before staring exploration
  //

  ros::Rate countdown(1);
  while ((ros::Time::now() - heartbeat).toSec() > 5.0) { // 5 second timeout
    ROS_INFO_THROTTLE(10, "[ExplorationManager] exploration not active");
    countdown.sleep();
    ros::spinOnce();
  }

  //
  // if the robots begin in close proximity, only the leader should begin by
  // capturing a panorama; if this robot is not the leader it should wait
  // to receive shared goals from the other agents
  //

  if (leader) {
    ROS_INFO("[ExplorationManager]: capturing initial panorama");
    capturePanorama();
  } else {
    while (shared_goals_count < robot_id) { // TODO: find a better way of doing this
      countdown.sleep(); countdown.sleep();
      ROS_INFO("[ExplorationManager]: robot%d is waiting for %d goals to be received before beginning exploration", robot_id, robot_id);
      ros::spinOnce();
    }
  }

  //
  // exploration loop
  //

  while (ros::ok()) {

    // get the latest information from the team and check if the exploration
    // loop should exit
    ros::spinOnce();
    if ((ros::Time::now() - heartbeat).toSec() > 5.0) { // 5 second timeout
      ROS_INFO("[ExplorationManager] exiting exploration loop");
      return;
    }

    //
    // Compute next panorama capture location
    //

    bool goal_found = false;
    grid_mapping::Point goal_pt;
    if (goal_method == "frontier") {
      goal_found = frontierGoal(goal_pt);
    } else {
      goal_found = csqmiGoal(goal_pt);
    }

    // if all goals are disqualified: wait for a new map and then start planning again
    if (!goal_found) {
      ROS_WARN("[ExplorationManager]: failed to find a goal: waiting for new map");
      received_new_map = false;
      while (!received_new_map) {
        countdown.sleep();
        ROS_INFO_THROTTLE(10, "[ExplorationManager]: waiting for a new map");
        ros::spinOnce();
      }
      continue;
    }

    //
    // Navigate to the goal point
    //

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = world_frame;
    target_pose.pose.position.x = goal_pt.x;
    target_pose.pose.position.y = goal_pt.y;
    target_pose.pose.orientation.w = 1.0;
    goal_pose_pub.publish(target_pose);

    scarab_msgs::MoveGoal action_goal;
    action_goal.target_poses.push_back(target_pose);

    // check if exploration is still active
    if ((ros::Time::now() - heartbeat).toSec() > 5.0) { // 5 second timeout
      ROS_INFO("[ExplorationManager] exiting exploration loop");
      return;
    }

    // NOTE if navigation fails, target_goal will not be cleared from other
    // robot's goal_locations vectors and the location will be avoided
    move_ac->sendGoal(action_goal, std::bind(&ExplorationManager::moveDoneCB, this, std::placeholders::_1, std::placeholders::_2), &moveActiveCB, &moveFeedbackCB);
    navigation_in_progress = true;
    //move_ac->waitForResult();
    while (navigation_in_progress) {
      if ((ros::Time::now() - heartbeat).toSec() > 5.0) { // 5 second timeout
        ROS_INFO("[ExplorationManager] exiting exploration loop");
        return;
      }
      ros::spinOnce();
      countdown.sleep();
    }

    if (!navigation_succeeded) {
      ROS_INFO("[ExplorationManager]: Navigation failed. Restarting planning process.");
      continue;
    }

    capturePanorama();

    ROS_INFO("[ExplorationManager]: Completed iteration loop %d", pan_count);
  }

}


bool hasUnknownNeighbor(const std::vector<int> cells,
                        const nav_msgs::OccupancyGrid& grid)
{
  for (int cell : cells)
    if (grid.data[cell] == 50)
      return true;
  return false;
}


std::vector<std::vector<grid_mapping::Point>> ExplorationManager::findFrontiers()
{
  // apply threshold
  nav_msgs::OccupancyGrid frontier_grid = *ang_grid.createROSOGMsg();
  nav_msgs::OccupancyGrid tmp_map = frontier_grid;
  for (signed char& cell : frontier_grid.data)
    if (cell < 50)
      cell = 0;
  for (int idx = 0; idx < frontier_grid.data.size(); ++idx) {
    if (tmp_map.data[idx] > 50) {
      frontier_grid.data[idx] = 100;
      for (int n : ang_grid.neighborIndices(idx, 0.3))
        frontier_grid.data[n] = 100;
    }
  }

  frontier_grid_pub.publish(frontier_grid);

  // find unknown cells bordering the free space connected to the robot
  std::vector<std::vector<grid_mapping::Point>> frontiers;
  std::stack<int> conn_free_cells; // free cells connected to the robot
  conn_free_cells.push(ang_grid.positionToIndex(pan_locations.back()));
  while (!conn_free_cells.empty()) {
    int cell = conn_free_cells.top();
    conn_free_cells.pop();
    frontier_grid.data[cell] = 100;

    for (int n : ang_grid.neighborIndices(cell)) {
      if (frontier_grid.data[n] == 0)
        conn_free_cells.push(n);
      else if (frontier_grid.data[n] == 50) {

        // travel along frontier
        std::stack<int> next_frontier_cells;
        next_frontier_cells.push(cell);
        std::vector<int> frontier;
        while (!next_frontier_cells.empty()) {
          int current_cell = next_frontier_cells.top();
          next_frontier_cells.pop();
          frontier_grid.data[current_cell] = 100;
          frontier.push_back(current_cell);

          // find neighbor frontier cells
          for (int n : ang_grid.neighborIndices(current_cell))
            if (frontier_grid.data[n] == 0)
              if (hasUnknownNeighbor(ang_grid.neighborIndices(n), frontier_grid))
                next_frontier_cells.push(n);
        }

        // ensure frontier is big enough
        if (frontier.size() > 1.0/ang_grid.resolution) {
          ROS_INFO("[ExplorationManager] found frontier with %ld cells", frontier.size());
          std::vector<grid_mapping::Point> pt_frontier;
          for (int cell : frontier)
            pt_frontier.push_back(ang_grid.indexToPosition(cell));
          frontiers.push_back(pt_frontier);
        }

        break; // exits: for (int n : ang_grid.neighborIndices(cell))
      }
    }
  }

  return frontiers;
}


} // namespace csqmi_exploration
