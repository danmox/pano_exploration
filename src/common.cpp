#include "csqmi_planning/common.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>

void displayImageComplement(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  double min, max;
  cv::minMaxLoc(img, &min, &max);
  cv::imshow(name, 255 - img*(255/max));
  cv::waitKey(0);
}

void displayImage(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  double min, max;
  cv::minMaxLoc(img, &min, &max);
  cv::imshow(name, img*(255/max));
  cv::waitKey(0);
}

cv::Mat loadMat(std::string file)
{
  std::ifstream f(file, std::ios::in | std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    std::cout << "Error opening map.bin" << std::endl;
    exit(-1);
  }

  size_t sz = f.tellg();
  std::vector<char> map_data(sz);
  f.seekg(0, std::ios::beg);
  f.read(reinterpret_cast<char*>(map_data.data()), sz);

  if (map_data.size() != 258*292) {
    std::cout << "Input data and Mat dimensions do not agree" << std::endl;
    exit(EXIT_FAILURE);
  }

  return cv::Mat(292, 258, CV_8UC1, map_data.data()).t();
}

typedef grid_mapping::OccupancyGridConstPtr GridConstPtr;
void occupancyGridToMat(const GridConstPtr& grid, cv::Mat& img)
{
  // convert logodds data to occupancy data [-inf,+inf] -> [0, 100]
  std::vector<int> map_data;
  std::transform(grid->data.begin(), grid->data.end(), 
      std::back_inserter(map_data), 
      [](double a){ return static_cast<int>(100*(1.0/(1.0+exp(-a)))) ;});

  // convert perspective (multi-layered) grid to 2D grid
  size_t layer_size = grid->width*grid->height;
  std::vector<int> map_data_2d(layer_size, 50);
  for (size_t i = 0; i < layer_size; ++i) {

    int min = 50, max = 50;
    for (int l = 0; l < grid->layers; ++l) {
      int val = map_data[i + l*layer_size];
      min = val < min ? val : min;
      max = val > max ? val : max;
    }

    // give preference to occupied cells
    if (max > 50)
      map_data_2d[i] = max;
    else if (min < 50)
      map_data_2d[i] = min;
  }

  // create cv::Mat from transformed grid data
  img = cv::Mat(map_data_2d).reshape(0, grid->height);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
}
