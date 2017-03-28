#include "grid_mapping/occ_grid.h"
#include <unordered_set>
#include <algorithm>
#include <iterator>

namespace grid_mapping {

OccGrid::OccGrid(Point origin_, double res, int w_, int h_, bool alloc_data) :
  GridBase(origin_, res, w_, h_)
{
  if (alloc_data)
    data = std::vector<double>(w*h, 0.0);
}

OccGrid::OccGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) :
  GridBase(Point(msg->info.origin.position.x, msg->info.origin.position.y),
      msg->info.resolution, msg->info.width, msg->info.height)
{
  data.reserve(msg->data.size());
  for (auto cell : msg->data)
    data.push_back(cell / 100.0);
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
      return std::vector<double>(ranges.size(), 0.0);

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

    if (range < scan->range_max)
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

template<typename Grid>
void OccGrid::insertROSGridMsg(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  Grid in(msg);
  insertMap(in);
}

template <typename Grid>
void OccGrid::insertMap(const Grid& in)
{
  // ensure current map spans input map
  if (!inBounds(in.origin) || !inBounds(in.topCorner()))
    expandMap(in.origin, in.topCorner());

  // update local grid with in_grid data
  update(&in);
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
