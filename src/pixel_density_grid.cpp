#include "grid_mapping/pixel_density_grid.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace grid_mapping {

PixelDensityGrid::PixelDensityGrid(Point origin_, double res, int w_, int h_) :
  OccGrid(origin_, res, w_, h_)
{
}

PixelDensityGrid::PixelDensityGrid(const OccupancyGrid::ConstPtr& msg) :
  OccGrid(msg)
{
}

// update existing map with other map info
void PixelDensityGrid::update(const PixelDensityGrid* grid)
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

// update existing map to have new dimensions
void PixelDensityGrid::update(const Point new_origin, const int w_new, 
    const int h_new)
{
  PixelDensityGrid new_grid(new_origin, resolution, w_new, h_new);
  new_grid.update(this);
  origin = new_origin;
  w = w_new;
  h = h_new;
  data = new_grid.data;
}

void PixelDensityGrid::insertScan(const sensor_msgs::LaserScanConstPtr& scan, 
    const geometry_msgs::Pose2DConstPtr& pose)
{
  // check if scan falls within the map; resize the map if necessary
  Point scan_origin(pose->x, pose->y);
  Point scan_bbx_min = scan_origin - scan->range_max;
  Point scan_bbx_max = scan_origin + scan->range_max;
  if (!inBounds(scan_bbx_min) || !inBounds(scan_bbx_max))
    expandMap(scan_bbx_min, scan_bbx_max);

  // update pixel hits
  double angle = pose->theta + scan->angle_min;
  int ranges_size = scan->ranges.size();
  std::vector<double> ranges = filterLaserScan(scan);
  for (double range : ranges) {
    if (range < scan->range_max && range > scan->range_min) {
      Point ray_end = scan_origin + range*Point(cos(angle), sin(angle));
      data[positionToIndex(ray_end)] += 1.0;
      angle += scan->angle_increment;
    }
  }
}

} // namespace grid_mapping
