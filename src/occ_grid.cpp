#include "grid_mapping/occ_grid.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>

#include <unordered_set>
#include <algorithm>
#include <iterator>

namespace grid_mapping {

OccGrid::OccGrid(Point origin_, double res, int w_, int h_, bool alloc_data) :
  GridBase(origin_, res, w_, h_), range_min(0.45), range_max(4.0)
{
  if (alloc_data)
    data = std::vector<double>(w*h, 0.0);
}

OccGrid::OccGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) :
  GridBase(Point(msg->info.origin.position.x, msg->info.origin.position.y),
      msg->info.resolution, msg->info.width, msg->info.height), 
  range_min(0.45), range_max(4.0)
{
  data.reserve(msg->data.size());
  for (auto cell : msg->data)
    data.push_back(cell / 100.0);
}

OccGrid::OccGrid(const grid_mapping::OccupancyGrid::ConstPtr& msg) :
  GridBase(msg), data(msg->data), range_min(0.45), range_max(4.0)
{
}

double OccGrid::cellProb(const int idx) const
{
  return 1.0-(1.0/(1.0 + exp(data[idx])));
}

// update existing map to have new dimensions
void OccGrid::update(const Point new_origin, const int w_new, const int h_new)
{
  OccGrid new_grid(new_origin, resolution, w_new, h_new);
  new_grid.update(this);
  origin = new_origin;
  w = w_new;
  h = h_new;
  data = new_grid.data;
}

// update existing map with other map info
void OccGrid::update(const OccGrid* grid)
{
  int w_in = grid->w;
  int origin_offset = positionToIndex(grid->origin);
  for (int i = 0; i < grid->h; ++i) {
    int c = origin_offset + i*w;
    int c_in = i*w_in;
    for (int j = 0; j < w_in; ++j) {
      data[c+j] += grid->data[c_in + j];
    }
  }
}

// Expand map to encompass p_min, p_max
void OccGrid::expandMap(const Point p_min, const Point p_max)
{
  // determine new extents of map, adding a padding of cells to attempt to
  // decrease the number of calls to expandMap(...), and subsequent copying
  Point new_origin(origin);
  Point new_top_corner = topCorner();
  double pad = round(0.2*std::max(w,h)) * resolution;
  if (p_min.x < new_origin.x)
    new_origin.x = roundToMapRes(p_min.x) - pad;
  if (p_min.y < new_origin.y)
    new_origin.y = roundToMapRes(p_min.y) - pad;
  if (p_max.x >= new_top_corner.x)
    new_top_corner.x = roundToMapRes(p_max.x) + pad;
  if (p_max.y >= new_top_corner.y)
    new_top_corner.y = roundToMapRes(p_max.y) + pad;
  int w_new = round((new_top_corner.x - new_origin.x) / resolution) + 1;
  int h_new = round((new_top_corner.y - new_origin.y) / resolution) + 1;

  // overwrite old map with new map
  update(new_origin, w_new, h_new);
}

std::vector<double> OccGrid::filterLaserScan(const sensor_msgs::
    LaserScanConstPtr& scan)
{
  std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());
  std::vector<double>::iterator it = ranges.begin(), end = ranges.end();
  while (it != end) {
    if (!std::isnan(*it)) {
      if (*it < scan->range_min)
        *it = 0.0;
      else if (*it > scan->range_max)
        *it = scan->range_max;

      ++it;
      continue;
    }

    // find all adjacent NANs
    auto nan_it = std::find_if(it, end, [](double a){return !std::isnan(a);});

    // if there are only a few adjacent NANs, discard those range values
    if (std::distance(it, nan_it) < 6) {
      std::fill(it, nan_it, 0.0);
      continue;
    }

    // try to guess if the large block of NANs should be 0.0 (within the 
    // sensor's minimum range) or scan->range_max (beyond the sensor's maximum
    // range) by checking for adjacent valid values
    double last_range = 0.0;
    if (nan_it != end) 
      last_range = *nan_it;
    else if (it != ranges.begin())
      last_range = *(it-1);
    else // the entire laser scan is NANs
      return std::vector<double>(ranges.size(), scan->range_max);

    // if the last valid range was closer to the sensor's maximum range than 
    // its minimum range, assume that the NANs should be the maximum range
    if (fabs(scan->range_max-last_range) < fabs(last_range-scan->range_min))
      std::fill(it, nan_it, scan->range_max);
    else
      std::fill(it, nan_it, 0.0);

    it = nan_it;
  }

  return ranges;
}

void OccGrid::insertScan(const sensor_msgs::LaserScanConstPtr& scan, 
    const geometry_msgs::Pose2DConstPtr& pose)
{
  // check if scan falls within the map; resize the map if necessary
  Point scan_origin(pose->x, pose->y);
  Point scan_bbx_min = scan_origin - scan->range_max;
  Point scan_bbx_max = scan_origin + scan->range_max;
  if (!inBounds(scan_bbx_min) || !inBounds(scan_bbx_max))
    expandMap(scan_bbx_min, scan_bbx_max);

  // initialize unordered_map to keep track of cells to update such that each
  // is updated only once
  std::unordered_set<int> occupied_cells, free_cells;

  // build lists of cells to update
  double angle = pose->theta + scan->angle_min;
  int ranges_size = scan->ranges.size();
  std::vector<double> ranges = filterLaserScan(scan);
  for (double range : ranges) {
    Point ray_end = scan_origin + range*Point(cos(angle), sin(angle));

    if (range < scan->range_max && range > scan->range_min)
      occupied_cells.emplace(positionToIndex(ray_end));

    for (int cell : rayCast(scan_origin, ray_end))
      free_cells.emplace(cell);

    angle += scan->angle_increment;
  }

  // give preference to occupied cells
  for (auto occ_cell : occupied_cells)
      free_cells.erase(occ_cell);

  // update occupied and free cells
  for (int cell : free_cells)
    data[cell] -= LOG_ODDS_FREE;
  for (int cell : occupied_cells)
    data[cell] += LOG_ODDS_OCCUPIED;
}

// update the map from a saved panorama rosbag
void OccGrid::insertPanorama(const std::string bagfile)
{
  rosbag::Bag bag;
  bag.open(bagfile, rosbag::bagmode::Read);

  // extract depth camera info
  sensor_msgs::CameraInfo camera_info;
  for (auto m : rosbag::View(bag, rosbag::TopicQuery("depth_camera_info"))) {
    auto msg = m.instantiate<sensor_msgs::CameraInfo>();
    if (msg) {
      camera_info = *msg;
      break;
    }
  }
  int img_w = camera_info.width;
  int img_h = camera_info.height;
  double Cx = camera_info.K[2];
  double fx = camera_info.K[0];
  double fy = camera_info.K[4];

  std::deque<geometry_msgs::TransformStamped> camera_trans;
  std::deque<sensor_msgs::Image> imgs;
  std::vector<std::string> topics = {"slam_camera_pose", "depth"};
  for (auto m : rosbag::View(bag, rosbag::TopicQuery(topics))) {
    if (m.getTopic().compare("slam_camera_pose") == 0)
      camera_trans.push_back(*m.instantiate<geometry_msgs::TransformStamped>());
    else if (m.getTopic().compare("depth") == 0)
      imgs.push_back(*m.instantiate<sensor_msgs::Image>());

    if (!camera_trans.empty() && !imgs.empty()) {
      // convert camera pose to 2D pose
      geometry_msgs::Pose2D pose;
      pose.x = camera_trans.front().transform.translation.x;
      pose.y = camera_trans.front().transform.translation.y;
      pose.theta = tf::getYaw(camera_trans.front().transform.rotation);
      geometry_msgs::Pose2DConstPtr pose_ptr(new geometry_msgs::Pose2D(pose));

      // compute offset in pixels between center row of depth image and row in
      // plane parallel to the ground (if camera is tilted up, we want to
      // extract the row of the depth image that points directly foward -- as
      // if the camera was not tilted)
      double roll, tilt_angle, yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(camera_trans.front().transform.rotation, quat);
      tf::Matrix3x3(quat).getRPY(roll, tilt_angle, yaw);
      int tilt_offset = -fy * tan(tilt_angle);

      sensor_msgs::LaserScan scan;
      scan.angle_min = -atan2((double)img_w-1.0-Cx, fx);
      scan.angle_max = atan2(Cx, fx);
      scan.angle_increment =(scan.angle_max-scan.angle_min)/((double)img_w-1.0);
      scan.time_increment = 0.0;
      scan.scan_time = 1.0/30.0; // 30Hz
      scan.range_min = range_min;
      scan.range_max = range_max;
      scan.header = imgs[0].header;
      scan.ranges.reserve(img_w);

      uint16_t* depth_row = reinterpret_cast<uint16_t*>(imgs[0].data.data());
      int row_offset = img_h/2 + tilt_offset;
      if (row_offset < 0 || row_offset >= img_w) {
        ROS_FATAL("OccGrid::insertPanorama(...): Row %d not in depth image with"
            " %d rows", row_offset, img_h);
        exit(EXIT_FAILURE);
      }
      depth_row += (row_offset*img_w);
      for (int u = img_w-1; u >= 0; --u) { // laserscans are logged CCW
        double z = depth_row[u] / 1000.0;
        double x = (u - Cx) * z / fx;
        double r = sqrt(pow(x, 2.0) + pow(z, 2.0));
        scan.ranges.push_back(r);
      }
      sensor_msgs::LaserScanConstPtr scan_ptr(new sensor_msgs::LaserScan(scan));

      // update map with laserscan
      insertScan(scan_ptr, pose_ptr);

      camera_trans.pop_front();
      imgs.pop_front();
    }
  }

  // extract panorama pose
  geometry_msgs::TransformStamped pan_pose;
  for (auto m : rosbag::View(bag, rosbag::TopicQuery("panorama_pose"))) {
    auto msg = m.instantiate<geometry_msgs::TransformStamped>();
    if (msg) {
      pan_pose = *msg;
      frame_id = msg->header.frame_id; // robotX/map
      break;
    }
  }

  // ensure robot origin is marked as free (because of the slight offset between
  // the pose of the robot and pose of the camera the exact position of the
  // robot sometimes is not marked as free in the resulting occupancy grid)
  Point po(pan_pose.transform.translation.x, pan_pose.transform.translation.y);
  updateRobotCells(po);

  bag.close();
}

void OccGrid::updateRobotCells(const Point po)
{
  int origin_cell = positionToIndex(po);
  auto robot_cells = neighborIndices(origin_cell, 0.1);
  robot_cells.push_back(origin_cell);
  for (auto cell : robot_cells)
    data[cell] -= 5.0; // sufficiently high log-odds free value
}

std::ostream& operator<<(std::ostream& out, const OccGrid& grid)
{
  std::cout << std::endl;
  std::cout << "info:" << std::endl;
  std::cout << "  origin: " << grid.origin << std::endl;
  std::cout << "  w: " << grid.w << std::endl;
  std::cout << "  h: " << grid.h << std::endl;
  std::cout << "  resolution: " << grid.resolution << std::endl;
  std::cout << "data:" << std::endl;
  for (int i = 0; i < grid.data.size(); ++i) {
    if (i % grid.w != 0)
      std::cout << grid.data[i] << ", ";
    else
      std::cout << std::endl << "  " << grid.data[i] << ", ";
  }
  std::cout << std::endl;
}

} // namespace grid_mapping
