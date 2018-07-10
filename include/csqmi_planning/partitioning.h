#ifndef CSQMI_PLANNING_PARTITIONING_H_
#define CSQMI_PLANNING_PARTITIONING_H_

#include <grid_mapping/angle_grid.h>
#include <opencv2/opencv.hpp>
#include <vector>

typedef std::vector<cv::Point> cv_points;

cv_points locatePanoramasOnSkeleton(const cv::Mat, cv_points);
std::vector<cv_points> partitionSkeleton(const cv::Mat, const cv_points);
std::vector<std::vector<int>> matPixelsToGridIndices(
    const std::vector<cv_points>&, const grid_mapping::AngleGrid&);

#endif
