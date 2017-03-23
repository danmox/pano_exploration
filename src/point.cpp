#include "grid_mapping/point.h"

namespace grid_mapping {

Point max(const Point& a, const Point& b)
{
  double x = a.x > b.x ? a.x : b.x;
  double y = a.y > b.y ? a.y : b.y;
  return Point(x, y);
}

Point min(const Point& a, const Point& b)
{
  double x = a.x < b.x ? a.x : b.x;
  double y = a.y < b.y ? a.y : b.y;
  return Point(x, y);
}

// add support for select lhs operations
Point operator *(const double lhs, const Point rhs) {return rhs*lhs;}
Point operator +(const double lhs, const Point rhs) {return rhs+lhs;}

} // namespace grid_mapping
