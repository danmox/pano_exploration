#ifndef GRID_MAPPING_OCC_GRID_H
#define GRID_MAPPING_OCC_GRID_H

#include "grid_mapping/grid_base.h"

#include <grid_mapping/OccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <iostream>
#include <string.h>

namespace grid_mapping {

#define LOG_ODDS_OCCUPIED 0.85
#define LOG_ODDS_FREE 0.5

class OccGrid : public GridBase
{
  public:
    std::vector<double> data;
    std::string frame_id;
    double range_min, range_max;

    double cellProb(const int) const;

    OccGrid(Point, double, int, int, bool = true);
    OccGrid(const nav_msgs::OccupancyGrid::ConstPtr&);
    OccGrid(const OccupancyGrid::ConstPtr&);

    virtual void update(const OccGrid*);
    virtual void update(const Point, const int, const int);

    virtual void updateRobotCells(const Point);

    void expandMap(const Point, const Point);
    /*
    template<typename Grid>
    void insertROSGridMsg(const nav_msgs::OccupancyGrid::ConstPtr&);
    template<typename Grid>
    void insertMap(const Grid&);
    */

    virtual void insertScan(const sensor_msgs::LaserScanConstPtr&,
        const geometry_msgs::Pose2DConstPtr&);
    void insertPanorama(const std::string);

    std::vector<double> filterLaserScan(const sensor_msgs::LaserScanConstPtr&);

    friend std::ostream& operator<<(std::ostream& out, const OccGrid& grid);
};

/*
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
*/

} // namespace grid_mapping

#endif
