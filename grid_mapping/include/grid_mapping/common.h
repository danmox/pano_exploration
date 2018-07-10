#ifndef GRID_MAPPING_COMMON_H_
#define GRID_MAPPING_COMMON_H_

#include "grid_mapping/pixel_density_grid.h"
#include "grid_mapping/angle_grid.h"
#include "grid_mapping/occ_grid.h"
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <string>

cv::Mat createGridImage(const grid_mapping::OccGrid& grid);
cv::Mat createGridImage(const grid_mapping::AngleGrid& grid, 
    const unsigned int layer=0);
cv::Mat createGridImage(const nav_msgs::OccupancyGrid& grid);
cv::Mat createGridImage(const grid_mapping::PixelDensityGrid& grid);
void displayImage(const cv::Mat& img, const std::string name);
void displayImageComplement(const cv::Mat& img, const std::string name);

#endif
