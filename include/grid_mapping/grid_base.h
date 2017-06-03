#ifndef GRID_MAPPING_GRID_BASE_H
#define GRID_MAPPING_GRID_BASE_H

#include "grid_mapping/point.h"

#include <grid_mapping/OccupancyGrid.h>
#include <vector>

namespace grid_mapping {

class GridBase
{
  public:
    // data members
    int w, h;
    double resolution;
    Point origin; // center of bottom-leftmost cell

    GridBase(Point, double, int, int);
    GridBase(const OccupancyGrid::ConstPtr&);

    double roundToMapRes(const double) const;
    Point roundToMapRes(const Point) const;

    // the origin of each grid cell is its center
    bool inBounds(const double, const double) const;
    bool inBounds(const Point) const;
    Point bbxMin() const; // takes into account resolution
    Point bbxMax() const; // takes into account resolution
    Point topCorner() const; // center of furthest cell from origin

    // index conversion methods for 2D grid stored as 1D array
    void indexToPosition(const int, double&, double&) const;
    Point indexToPosition(const int) const;
    bool indexToPositionChecked(const int, double&, double&) const;
    bool indexToPositionChecked(int, Point&) const;
    int positionToIndex(const double, const double) const;
    int positionToIndex(const Point) const;
    bool positionToIndexChecked(const double, const double, int&) const;
    bool positionToIndexChecked(const Point, int&) const;

    // neighbor cell methods
    std::vector<int> neighborIndices(const int cell, const int rad=1) const;
    std::vector<int> neighborIndices(const int, const double) const;
    
    // raycasting methods
    void bbxIntersection(const Point, Point&) const;
    std::vector<int> rayCast(const Point, Point) const;
    std::vector<int> rayCast(const int, const int) const;
};

} // namespace grid_mapping

#endif
