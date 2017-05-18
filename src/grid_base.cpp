#include "grid_mapping/grid_base.h"
#include <ros/ros.h>

namespace grid_mapping {

GridBase::GridBase(Point origin_, double res, int w_, int h_) :
  origin(origin_), resolution(res), w(w_), h(h_)
{
}

double GridBase::roundToMapRes(const double num) const
{
  return round(num / resolution) * resolution;
}

Point GridBase::roundToMapRes(const Point p) const
{
  return Point(roundToMapRes(p.x), roundToMapRes(p.y));
}

// the origin of each cell is at its center; points on the south 
// and west cell sides are included
bool GridBase::inBounds(const double x, const double y) const
{
  double x_min, y_min, x_max, y_max;
  (origin - resolution/2.0).components(x_min, y_min);
  x_max = x_min + w*resolution;
  y_max = y_min + h*resolution;

  if (x < x_min || x >= x_max)
    return false;
  if (y < y_min || y >= y_max)
    return false;

  return true;
}

bool GridBase::inBounds(const Point p) const
{
  return inBounds(p.x, p.y);
}

Point GridBase::bbxMin() const
{
  return origin - resolution/2.0;
}

Point GridBase::bbxMax() const
{
  return origin + Point(w*resolution, h*resolution) - resolution/2.0;
}

Point GridBase::topCorner() const
{
  return indexToPosition(w*h-1);
}

void GridBase::indexToPosition(const int cell, double& x, double& y) const
{
  x = (cell % w) * resolution + origin.x;
  y = (cell / w) * resolution + origin.y; 
}

Point GridBase::indexToPosition(const int cell) const
{
    return Point((cell % w) * resolution + origin.x, 
        (cell / w) * resolution + origin.y);
}

bool GridBase::indexToPositionChecked(const int cell, double& x, double& y) 
  const
{
  if (cell < 0 || cell > w*h-1)
    return false;

  indexToPosition(cell, x, y);
  return true;
}

bool GridBase::indexToPositionChecked(const int cell, Point& p) const
{
  if (cell < 0 || cell > w*h-1)
    return false;

  p = indexToPosition(cell);
  return true;
}

int GridBase::positionToIndex(const double x, const double y) const
{
  return round((x-origin.x)/resolution) + 
    round((y-origin.y)/resolution)*w;
}

int GridBase::positionToIndex(const Point p) const
{
  return positionToIndex(p.x, p.y);
}

bool GridBase::positionToIndexChecked(const double x, const double y, 
    int& index) const
{
  if (!inBounds(x, y))
    return false;

  index = positionToIndex(x,y);
  return true;
}

bool GridBase::positionToIndexChecked(const Point p, int& index) const
{
  return positionToIndexChecked(p.x, p.y, index);
}

// neighbor cell methods
typedef std::vector<int> int_vec;

int_vec GridBase::neighborIndices(const int cell, const int rad) const
{
  // determine 2D indices
  int center_c = cell % w;
  int center_r = cell / w;
  int c_min = center_c - rad;
  int c_max = center_c + rad;
  int r_min = center_r - rad;
  int r_max = center_r + rad;

  // ensure index ranges are in bounds
  if (c_min < 0)
    c_min = 0;
  if (c_max >= w)
    c_max = w-1;
  if (r_min < 0)
    r_min = 0;
  if (r_max >= h)
    r_max = h-1;

  // compute linear indices
  int_vec neighbors;
  for (int r = r_min; r <= r_max; ++r) {
    for (int c = c_min; c <= c_max; ++c) {
      int index = c + r*w;
      if (index != cell)
        neighbors.push_back(index);
    }
  }

  return neighbors;
}

int_vec GridBase::neighborIndices(const int cell, const double rad) const
{
  return neighborIndices(cell, (int)(roundToMapRes(rad)/resolution));
}

void GridBase::bbxIntersection(const Point p1, Point& p2) const
{
  // make sure the start point is inside the bbx and end point is not
  if (!inBounds(p1)) {
    Point bbx_min = bbxMin();
    Point bbx_max = bbxMax();
    ROS_FATAL_STREAM("GridBase::bbxIntersection(...): p1 " << p1 << " outside "
        "of current map bounds: " << bbxMin() << " " << bbxMax());
    exit(EXIT_FAILURE);
  } 
  if (inBounds(p2))
    return;

  // To ensure intersection point falls within map, the bounding box
  // used is the rectangle with corners at the origin of each corner cell
  double x0, x1;
  double y0, y1;
  origin.components(x0, y0);
  topCorner().components(x1, y1);

  // compute ray origin and direction
  double r0x, r0y;
  p1.components(r0x, r0y);
  double dx = p2.x - r0x;
  double dy = p2.y - r0y;

  double t0x = (x0 - r0x) / dx;
  double t1x = (x1 - r0x) / dx;
  double t0y = (y0 - r0y) / dy;
  double t1y = (y1 - r0y) / dy;

  double max_tx = t0x > t1x ? t0x : t1x;
  double max_ty = t0y > t1y ? t0y : t1y;
  double t = max_tx < max_ty ? max_tx : max_ty;

  p2 = Point(r0x + t*dx, r0y + t*dy);
}

int_vec GridBase::rayCast(const Point p1, Point p2) const
{
  if (!inBounds(p1)) {
    ROS_FATAL_STREAM("GridBase::rayCast(...): p1 " << p1 << " outside map with "
        "bounds: " << bbxMin() << " " << bbxMax());
    exit(EXIT_FAILURE);
  }

  int_vec indices;
  if (!inBounds(p2))
    bbxIntersection(p1, p2);

  const Point dp = p2 - p1;
  const double steps = round(dp.abs().max() / resolution);
  const Point ds = dp / steps; // Point(dx, dy)

  int it = 0;
  Point current_position = p1;
  while (it++ <= steps) {
    indices.push_back(positionToIndex(current_position));
    current_position += ds;
  }

  return indices;
}

int_vec GridBase::rayCast(const int start_index, const int end_index) const
{
  double x0, x1, y0, y1;
  if (!indexToPositionChecked(start_index, x0, y0)) {
    ROS_FATAL("GridBase::rayCast(...): start_index %d invalid", start_index);
    exit(EXIT_FAILURE);
  }
  if (!indexToPositionChecked(end_index, x1, y1)) {
    ROS_FATAL("GridBase::rayCast(...): end_index %d invalid", end_index);
    exit(EXIT_FAILURE);
  }

  return rayCast(Point(x0, y0), Point(x1, y1));
}

} // namespace grid_mapping
