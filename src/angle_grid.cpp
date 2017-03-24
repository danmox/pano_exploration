#include "grid_mapping/angle_grid.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iterator>
#include <cmath>

namespace grid_mapping {

AngleGrid::AngleGrid(Point origin_, double res, int w_, int h_, 
    int layers_) :
  GridBase(origin_, res, w_, h_),
  data(w_*h_*l_, 0.0),
  layers(l_)
{
  double step = 2.0*M_PI/double(layers);
  for (int i = 0; i < layers; ++i)
    bins.push_back(i*step);
}

AngleGrid::AngleGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) :
  GridBase(Point(msg->info.origin.position.x, msg->info.origin.position.y),
      msg->info.resolution, msg->info.width, msg->info.height)
{
  layers = msg->data.size() / (w*h);
  double step = 2.0*M_PI/double(layers);
  for (int i = 0; i < layers; ++i)
    bins.push_back(i*step);

  data.reserve(msg->data.size());
  for (auto cell : msg->data)
    data.push_back(cell / 100.0);
}

void AngleGrid::update(const Point new_origin, const int w_new, const int h_new)
{
  OccGrid new_grid(new_origin, resolution, w_new, h_new, layers);
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
	int w_in = grid->w;
	int origin_offset = positionToIndex(grid->origin);
	for (int i = 0; i < grid->w; ++i) {
		int c = origin_offset + i*w;
		int c_in = i*w_in;
		for (int j = 0; j < w_in; ++j) {
			for (int k = 0; k < layers; ++k) {
				int index_in = c_in + j + k*layer_in;
				int index = c + j + k*layer;
				data[index] += grid->data[index_in];
			}
		}
	}
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
  std::unordered_set<int> free_cells;
  std::unordered_map<int,int> occupied_cells;

  // build lists of cells to update
  double angle = pose->theta + scan->angle_min;
  int ranges_size = scan->ranges.size();
  std::vector<double> ranges = filterLaserScan(scan);
  for (double range : ranges) {
    Point ray_end = scan_origin + range*Point(cos(angle), sin(angle));

    if (range < scan->range_max && range > 0.0)
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

friend std::ostream& operator<<(std::ostream& out, const AngleGrid& grid);

} // namespace grid_mapping
