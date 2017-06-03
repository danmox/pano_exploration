#ifndef CSQMI_PLANNING_PARTITIONING_H_
#define CSQMI_PLANNING_PARTITIONING_H_

#include <opencv2/opencv.hpp>
#include <vector>

void locatePanoramasOnSkeleton(const cv::Mat, std::vector<cv::Point>&);

#endif
