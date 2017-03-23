#ifndef GRID_MAPPING_POINT_H
#define GRID_MAPPING_POINT_H

#include <math.h>
#include <iostream>

namespace grid_mapping {

class Point
{
  public:
    double x, y;

    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}

    void components(double& x_, double& y_) const {x_ = x; y_ = y;}
    
    double norm() const {sqrt(pow(x, 2.0) + pow(y, 2.0));}
    double max() const {return x > y ? x : y;}
    Point abs() const {return Point(fabs(x), fabs(y));}

    Point operator +(const double c) const {return Point(x+c, y+c);}
    Point operator +(const Point rhs) const {return Point(x+rhs.x, y+rhs.y);}
    Point operator -(const double c) const {return Point(x-c, y-c);}
    Point operator -(const Point rhs) const {return Point(x-rhs.x, y-rhs.y);}
    Point operator /(const double c) const {return Point(x/c, y/c);}
    Point operator *(const double c) const {return Point(x*c, y*c);}

    bool operator <=(const Point rhs) const {return x <= rhs.x && y <= rhs.y;}
    bool operator <(const Point rhs) const {return x < rhs.x && y < rhs.y;}
    bool operator >=(const Point rhs) const {return x >= rhs.x && y >= rhs.y;}
    bool operator >(const Point rhs) const {return x > rhs.x && y > rhs.y;}

    void operator +=(const Point rhs) {x += rhs.x; y += rhs.y;}
    void operator +=(const double c) {x += c; y += c;}
    void operator -=(const Point rhs) {x -= rhs.x; y -= rhs.y;}
    void operator -=(const double c) {x -= c; y -= c;}

    friend std::ostream& operator<<(std::ostream& out, const Point p)
    {
      out << "(" << p.x << ", " << p.y << ")";
      return out;
    }
};

Point max(const Point& a, const Point& b);
Point min(const Point& a, const Point& b);

// add support for select lhs operations
Point operator *(const double lhs, const Point rhs);
Point operator +(const double lhs, const Point rhs);

} // namespace grid_mapping

#endif
