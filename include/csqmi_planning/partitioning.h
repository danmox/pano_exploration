#ifndef CSQMI_PLANNING_PARTITIONING_H_
#define CSQMI_PLANNING_PARTITIONING_H_

#include <opencv2/opencv.hpp>
#include <vector>

typedef std::vector<cv::Point> cv_points;

cv_points locatePanoramasOnSkeleton(const cv::Mat, const cv_points);

#endif
