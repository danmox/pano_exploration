#ifndef GRID_MAPPING_OCC_GRID_H
#define GRID_MAPPING_OCC_GRID_H

#include "grid_mapping/grid_base.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <iostream>

namespace grid_mapping {

#define LOG_ODDS_OCCUPIED 0.85
#define LOG_ODDS_FREE 0.5

class OccGrid : public GridBase
{
  public:
    std::vector<double> data;

    OccGrid(Point, double, int, int);
    OccGrid(const nav_msgs::OccupancyGrid::ConstPtr&);

    virtual void update(const OccGrid*);
    virtual void update(const Point, const int, const int);

    void expandMap(const Point, const Point);
    template<typename Grid>
    void insertROSGridMsg(const nav_msgs::OccupancyGrid::ConstPtr&);
    template<typename Grid>
    void insertMap(const Grid&);

    void insertScan(const sensor_msgs::LaserScanConstPtr&,
        const geometry_msgs::Pose2DConstPtr&);

    std::vector<double> filterLaserScan(const sensor_msgs::LaserScanConstPtr&);

    friend std::ostream& operator<<(std::ostream& out, const OccGrid& grid);
};

} // namespace grid_mapping

#endif
