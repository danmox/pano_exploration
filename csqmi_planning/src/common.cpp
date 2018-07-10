#include "csqmi_planning/common.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>

void displayImageComplement(const cv::Mat& img, const std::string name)
{
  double min, max;
  cv::minMaxLoc(img, &min, &max);
  displayRawImage(255 - img*(255/max), name);
}

void displayImage(const cv::Mat& img, const std::string name)
{
  double min, max;
  cv::minMaxLoc(img, &min, &max);
  displayRawImage(img*(255/max), name);
}

void displayRawImage(const cv::Mat& img, const std::string name)
{
  static int save_count = 0;
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, img);
  char key = (char)cv::waitKey(0);

  if (key == 115) { // 's' for save
    std::string filename = "image" + std::to_string(++save_count) + ".yaml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "Mat" << img;
    fs.release();
    std::cout << "Wrote image to file: " << filename.c_str() << std::endl;
  }
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

void pixelDensityGridToMat(const grid_mapping::PixelDensityGrid& grid,
    cv::Mat& img)
{
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(),
      std::back_inserter(map_data),
      [](unsigned char a){ return static_cast<int>(a*255.0/100.0); });
  img = cv::Mat(map_data).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
}

void occupancyGridToMat(const GridConstPtr& grid, cv::Mat& img, const int layer)
{
  grid_mapping::AngleGrid ang_grid(grid);
  occupancyGridToMat(ang_grid, img, layer);
}

void occupancyGridToMat(const grid_mapping::AngleGrid& grid, cv::Mat& img,
    const int layer)
{
  // convert logodds data to occupancy data [-inf,+inf] -> [0, 100]
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(), 
      std::back_inserter(map_data), 
      [](double a){ return static_cast<int>(100*(1.0/(1.0+exp(-a)))) ;});

  // convert perspective (multi-layered) grid to 2D grid
  size_t layer_size = grid.w*grid.h;
  std::vector<int> map_data_2d;
  if (layer == -1) {
    map_data_2d = std::vector<int>(layer_size, 50);
    for (size_t i = 0; i < layer_size; ++i) {

      int min = 50, max = 50;
      for (int l = 0; l < grid.layers; ++l) {
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
  } else if (layer >= 0 && layer < grid.layers) {
    auto beg_it = map_data.begin() + layer*layer_size;
    map_data_2d = std::vector<int>(beg_it, beg_it+layer_size);
  } else {
    std::cout << "occupancyGridToMat(...): invalid layer " << layer <<std::endl;
    exit(EXIT_FAILURE);
  }

  // create cv::Mat from transformed grid data
  img = cv::Mat(map_data_2d).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
  //cv::flip(img, img, 0);
}
