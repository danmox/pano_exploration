#include "csqmi_planning/common.h"
#include "csqmi_planning/thinning.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grid_mapping/OccupancyGrid.h>

int main(int argc, char** argv)
{
  // read OccupancyGrid from saved bag
  cv::Mat img;
  rosbag::Bag bag;
  bag.open("data/pan_grid.bag", rosbag::bagmode::Read);
  grid_mapping::OccupancyGridConstPtr grid;
  for (auto m : rosbag::View(bag)) {
    grid = m.instantiate<grid_mapping::OccupancyGrid>();
    if (grid) {
      occupancyGridToMat(grid, img);
      break;
    }
  }

  // show image
  cv::Mat skeleton, color_img;
  computeSkeleton(img, skeleton);
  cv::cvtColor(255 - img*255.0/100.0, color_img, CV_GRAY2RGB);
  color_img.setTo(cv::Scalar(255, 0, 0), skeleton);
  displayRawImage(color_img, "raw image");

  return 0;
}
