#ifndef COMMON_H
#define COMMON_H

#include <opencv2/opencv.hpp>
#include <grid_mapping/OccupancyGrid.h>
#include <grid_mapping/angle_grid.h>
#include <string.h>

void displayImageComplement(const cv::Mat& img, const std::string name);
void displayImage(const cv::Mat& img, const std::string name);
void displayRawImage(const cv::Mat& img, const std::string name);
cv::Mat loadMat(std::string file);
void occupancyGridToMat(const grid_mapping::OccupancyGridConstPtr&, cv::Mat&,
    const int layer = -1);
void occupancyGridToMat(const grid_mapping::AngleGrid&, cv::Mat&,
    const int layer = -1);

#endif
