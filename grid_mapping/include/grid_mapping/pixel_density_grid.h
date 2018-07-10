#ifndef GRID_MAPPING_PIXEL_DENSITY_GRID_H_
#define GRID_MAPPING_PIXEL_DENSITY_GRID_H_

#include "grid_mapping/occ_grid.h"

namespace grid_mapping {

class PixelDensityGrid : public OccGrid
{
  public:
    PixelDensityGrid(Point, double, int, int);
    PixelDensityGrid(const OccupancyGrid::ConstPtr&);

    virtual void update(const PixelDensityGrid*);
    virtual void update(const Point, const int, const int);

    virtual void updateRobotCells(const Point) {} // do nothing

    virtual void insertScan(const sensor_msgs::LaserScanConstPtr&,
        const geometry_msgs::Pose2DConstPtr&);
};

} // namespace grid_mapping

#endif
