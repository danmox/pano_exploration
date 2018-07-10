#include "grid_mapping/common.h"

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <iterator>
#include <algorithm>

cv::Mat createGridImage(const nav_msgs::OccupancyGrid& grid)
{
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(),
      std::back_inserter(map_data),
      [](unsigned char a){ return static_cast<int>(a*255.0/100.0); });
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.info.height);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}

cv::Mat createGridImage(const grid_mapping::PixelDensityGrid& grid)
{
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(),
      std::back_inserter(map_data),
      [](double a){ return static_cast<int>(a); });
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}

cv::Mat createGridImage(const grid_mapping::OccGrid& grid)
{
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(), 
      std::back_inserter(map_data), 
      [](double a){ return static_cast<int>(255*(1.0/(1.0+exp(-a)))) ;});
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}

cv::Mat createGridImage(const grid_mapping::AngleGrid& grid, 
    const unsigned int layer)
{
  if (layer >= grid.layers) {
    ROS_FATAL("createGridImage(...): invalid layer argument");
    exit(EXIT_FAILURE);
  }
  const int layer_size = grid.w*grid.h;
  std::vector<int> map_data;
  std::transform(grid.data.begin() + layer*layer_size, 
      grid.data.begin() + (layer+1)*layer_size, 
      std::back_inserter(map_data), 
      [](double a){ return static_cast<int>(255*(1.0/(1.0+exp(-a)))) ;});
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}

void displayImage(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, img);
  cv::waitKey(0);
}

void displayImageComplement(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, 255-img);
  cv::waitKey(0);
}
