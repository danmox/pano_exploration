#ifndef GRID_MAPPING_POINT_H
#define GRID_MAPPING_POINT_H

#include <math.h>
#include <iostream>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

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

    static Point transformPoint(const geometry_msgs::TransformStamped& tfs,
        const Point p)
    {
      // assuming 2D: compute the translation and rotation of the transform
      double tx = tfs.transform.translation.x;
      double ty = tfs.transform.translation.y;
      double theta = tf::getYaw(tfs.transform.rotation);

      // apply transformation
      Point p_out;
      p_out.x = p.x*cos(theta) - p.y*sin(theta) + tx;
      p_out.y = p.x*sin(theta) + p.y*cos(theta) + ty;

      return p_out;
    }
};

Point max(const Point& a, const Point& b);
Point min(const Point& a, const Point& b);

// add support for select lhs operations
Point operator *(const double lhs, const Point rhs);
Point operator +(const double lhs, const Point rhs);

} // namespace grid_mapping

#endif
