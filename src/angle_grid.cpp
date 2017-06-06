#include "grid_mapping/angle_grid.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include <unordered_set>
#include <unordered_map>
#include <deque>
#include <algorithm>
#include <iterator>
#include <cmath>

namespace grid_mapping {

AngleGrid::AngleGrid(Point origin_, double res, int w_, int h_, int l_) :
  OccGrid(origin_, res, w_, h_, false),
  layers(l_)
{
  ros::Time::init(); // for timestamping ros msgs in createROSMsg()

  // initialize bins of angle grid
  double step = 2.0*M_PI/double(layers);
  for (int i = 0; i < layers; ++i)
    bins.push_back(-M_PI + i*step);

  data = std::vector<double>(w*h*layers, 0.0);
}

AngleGrid::AngleGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) :
  OccGrid(msg)
{
  ros::Time::init(); // for timestamping ros msgs in createROSMsg()

  layers = msg->data.size() / (w*h);
  double step = 2.0*M_PI/double(layers);
  for (int i = 0; i < layers; ++i)
    bins.push_back(i*step);
}

AngleGrid::AngleGrid(const grid_mapping::OccupancyGrid::ConstPtr& msg) :
  OccGrid(msg),
  layers(msg->layers),
  bins(msg->bins)
{
}

int AngleGrid::angleIndex(double angle)
{
  // the angle at which a ray intercepts a cell is the opposite of the angle of
  // the ray, constrained to (-pi pi]
  angle += M_PI;
  while (angle <= -M_PI)
    angle += 2.0*M_PI;
  while (angle > M_PI)
    angle -= 2.0*M_PI;

  int index = 0;
  while (angle > bins[index] && index < layers)
    ++index;

  return index-1;
}

void AngleGrid::update(const Point new_origin, const int w_new, const int h_new)
{
  AngleGrid new_grid(new_origin, resolution, w_new, h_new, layers);
  new_grid.update(this);
  origin = new_origin;
  w = w_new;
  h = h_new;
  data = new_grid.data;
}

void AngleGrid::update(const AngleGrid* grid)
{
	int layer_in = grid->w*grid->h;
	int layer = w*h;
	int origin_offset = positionToIndex(grid->origin);
	for (int i = 0; i < grid->h; ++i) {
		int c = origin_offset + i*w;
		int c_in = i*grid->w;
		for (int j = 0; j < grid->w; ++j) {
			for (int k = 0; k < layers; ++k) {
				int index_in = c_in + j + k*layer_in;
				int index = c + j + k*layer;
				data[index] += grid->data[index_in];
			}
		}
	}
}

void AngleGrid::insertScan(const sensor_msgs::LaserScanConstPtr& scan, 
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
  std::unordered_set<int> free_cells;
  std::unordered_map<int,int> occupied_cells;

  // build lists of cells to update
  double angle = pose->theta + scan->angle_min;
  int ranges_size = scan->ranges.size();
  std::vector<double> ranges = filterLaserScan(scan);
  for (double range : ranges) {
    Point ray_end = scan_origin + range*Point(cos(angle), sin(angle));

    if (range < scan->range_max && range > 0.0)
      occupied_cells.emplace(positionToIndex(ray_end), angleIndex(angle));

    for (int cell : rayCast(scan_origin, ray_end))
      for (int l = 0; l < layers; ++l)
        free_cells.emplace(cell + l*w*h);

    angle += scan->angle_increment;
  }

  // give preference to occupied cells
  for (auto occ_cell : occupied_cells)
      free_cells.erase(occ_cell.first);

  // update occupied and free cells
  for (int cell : free_cells)
    data[cell] -= LOG_ODDS_FREE;
  int d = w*h;
  for (auto cell_pair : occupied_cells) // pair: linear index, layer index
    data[cell_pair.first + d*cell_pair.second] += LOG_ODDS_OCCUPIED;
}

// update the map from a saved panorama rosbag
void AngleGrid::insertPanorama(const std::string bagfile)
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

  std::deque<geometry_msgs::PoseStamped> camera_poses;
  std::deque<sensor_msgs::Image> imgs;
  std::vector<std::string> topics = {"camera_pose", "depth"};
  for (auto m : rosbag::View(bag, rosbag::TopicQuery(topics))) {
    if (m.getTopic().compare("camera_pose") == 0)
      camera_poses.push_back(*m.instantiate<geometry_msgs::PoseStamped>());
    else if (m.getTopic().compare("depth") == 0)
      imgs.push_back(*m.instantiate<sensor_msgs::Image>());

    if (!camera_poses.empty() && !imgs.empty()) {
      // convert camera pose to 2D pose
      geometry_msgs::Pose2D pose;
      pose.x = camera_poses.front().pose.position.x;
      pose.y = camera_poses.front().pose.position.y;
      pose.theta = tf::getYaw(camera_poses.front().pose.orientation);
      geometry_msgs::Pose2DConstPtr pose_ptr(new geometry_msgs::Pose2D(pose));

      // compute offset in pixels between center row of depth image and row in 
      // plane parallel to the ground (if camera is tilted up, we want to 
      // extract the row of the depth image that points directly foward -- as 
      // if the camera was not tilted)
      double roll, tilt_angle, yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(camera_poses.front().pose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(roll, tilt_angle, yaw);
      int tilt_offset = -fy * tan(tilt_angle);

      sensor_msgs::LaserScan scan;
      scan.angle_min = -atan2((double)img_w-1.0-Cx, fx);
      scan.angle_max = atan2(Cx, fx);
      scan.angle_increment =(scan.angle_max-scan.angle_min)/((double)img_w-1.0);
      scan.time_increment = 0.0;
      scan.scan_time = 1.0/30.0; // 30Hz
      scan.range_min = 0.45; // Xtion min range
      scan.range_max = 4.0; // Xtion max range
      scan.header = imgs[0].header;
      scan.header.frame_id = "world";
      scan.ranges.reserve(img_w);

      uint16_t* depth_row = reinterpret_cast<uint16_t*>(imgs[0].data.data());
      int row_offset = img_h/2 + tilt_offset;
      if (row_offset < 0 || row_offset >= img_w) {
        ROS_FATAL("Row %d not in depth image with %d rows", row_offset, img_h);
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

      camera_poses.pop_front();
      imgs.pop_front();
    }
  }

  // extract panorama pose
  geometry_msgs::TransformStamped pan_pose;
  for (auto m : rosbag::View(bag, rosbag::TopicQuery("panorama_pose"))) {
    auto msg = m.instantiate<geometry_msgs::TransformStamped>();
    if (msg) {
      pan_pose = *msg;
      break;
    }
  }

  // ensure robot origin is marked as free (because of the slight offset between
  // the pose of the robot and pose of the camera the exact position of the
  // robot sometimes is not marked as free in the resulting occupancy grid)
  Point po(pan_pose.transform.translation.x, pan_pose.transform.translation.y);
  int origin_cell = positionToIndex(po);
  auto robot_cells = neighborIndices(origin_cell, 0.1);
  robot_cells.push_back(origin_cell);
  for (auto cell : robot_cells)
    for (int l = 0; l < layers; ++l)
      data[cell + l*w*h] -= 5.0; // sufficiently high log-odds free value

  bag.close();
}

OccupancyGridPtr AngleGrid::createROSMsg()
{
  static int seq = 0;

  OccupancyGridPtr grid(new OccupancyGrid);

  // make sure ros::Time is running
  if (!ros::Time::isValid())
    ros::Time::init();

  grid->header.seq = seq++;
  grid->header.stamp = ros::Time::now();
  grid->resolution = resolution;
  grid->width = w;
  grid->height = h;
  grid->layers = layers;
  grid->bins = bins;

  grid->origin.x = origin.x;
  grid->origin.y = origin.y;

  grid->data = data;

  return grid;
}

} // namespace grid_mapping
