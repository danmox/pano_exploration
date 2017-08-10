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

int AngleGrid::angleIndex(double angle) const
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

void AngleGrid::updateRobotCells(const Point po)
{
  int origin_cell = positionToIndex(po);
  auto robot_cells = neighborIndices(origin_cell, 0.1);
  robot_cells.push_back(origin_cell);
  for (auto cell : robot_cells)
    for (int l = 0; l < layers; ++l)
      data[cell + l*w*h] -= 5.0; // sufficiently high log-odds free value
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

    if (range < scan->range_max && range > scan->range_min)
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

void AngleGrid::insertMap(const OccupancyGridConstPtr& msg)
{
  AngleGrid in(msg);

  // ensure current map spans input map
  if (!inBounds(in.origin) || !inBounds(in.topCorner()))
    expandMap(in.origin, in.topCorner());

  // update local grid with in_grid data
  update(&in);
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
  grid->header.frame_id = frame_id;
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

nav_msgs::OccupancyGridPtr AngleGrid::createROSOGMsg()
{
  static int seq = 0;

  nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);

  // make sure ros::Time is running
  if (!ros::Time::isValid())
    ros::Time::init();

  grid->header.seq = seq++;
  grid->header.stamp = ros::Time::now();
  grid->header.frame_id = frame_id;
  grid->info.resolution = resolution;
  grid->info.width = w;
  grid->info.height = h;
  grid->info.origin.position.x = origin.x;
  grid->info.origin.position.y = origin.y;

  // convert perspective (multi-layered) grid to 2D grid
  size_t layer_size = w*h;
  grid->data = std::vector<signed char>(layer_size, 50);
  for (size_t i = 0; i < layer_size; ++i) {

    int min = 50, max = 50;
    for (int l = 0; l < layers; ++l) {
      int val = 100*cellProb(i + l*layer_size);
      min = val < min ? val : min;
      max = val > max ? val : max;
    }

    // give preference to occupied cells
    if (max > 50)
      grid->data[i] = max;
    else if (min < 50)
      grid->data[i] = min;
  }

  return grid;
}

} // namespace grid_mapping
