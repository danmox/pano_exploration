#ifndef GRID_MAPPING_COMMON_H_
#define GRID_MAPPING_COMMON_H_

#include "grid_mapping/angle_grid.h"
#include "grid_mapping/occ_grid.h"
#include <opencv2/opencv.hpp>
#include <string>

cv::Mat createGridImage(const grid_mapping::OccGrid& grid);
cv::Mat createGridImage(const grid_mapping::AngleGrid& grid, 
    const unsigned int layer=0);
void displayImage(const cv::Mat& img, const std::string name);
void displayImageComplement(const cv::Mat& img, const std::string name);

#endif
