#ifndef GRID_MAPPING_ANGLE_GRID_H
#define GRID_MAPPING_ANGLE_GRID_H

#include "grid_mapping/occ_grid.h"

#include <grid_mapping/OccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <string.h>
#include <iostream>

namespace grid_mapping {

class AngleGrid : public OccGrid
{
  public:
    int layers;
    std::vector<double> bins;

    AngleGrid(Point, double, int, int, int = 4);
    AngleGrid(const nav_msgs::OccupancyGrid::ConstPtr&);
    AngleGrid(const OccupancyGrid::ConstPtr&);

    virtual void update(const AngleGrid*);
    virtual void update(const Point, const int, const int);

    int angleIndex(double);
    void insertScan(const sensor_msgs::LaserScanConstPtr&,
        const geometry_msgs::Pose2DConstPtr&);
    void insertPanorama(const std::string);

    OccupancyGridPtr createROSMsg();
};

} // namespace grid_mapping

#endif
