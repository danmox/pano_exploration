#ifndef COMMON_H
#define COMMON_H

#include <opencv2/opencv.hpp>
#include <string.h>

void displayImageComplement(const cv::Mat& img, const std::string name);
void displayImage(const cv::Mat& img, const std::string name);
cv::Mat loadMat(std::string file);

#endif
