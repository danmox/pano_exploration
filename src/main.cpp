#include "csqmi_planning/common.h"
#include "csqmi_planning/thinning.hpp"
#include "csqmi_planning/partitioning.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grid_mapping/angle_grid.h>

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
  grid_mapping::AngleGrid ang_grid(grid);

  // show image
  cv::Mat skeleton, color_img;
  computeSkeleton(img, skeleton);
  cv::cvtColor(255 - img*255.0/100.0, color_img, CV_GRAY2RGB);
  color_img.setTo(cv::Scalar(255, 0, 0), skeleton);
  int origin_idx = ang_grid.positionToIndex(grid_mapping::Point(0.0,0.0));
  std::vector<cv::Point> pans;
  pans.push_back(cv::Point(origin_idx % ang_grid.w, origin_idx / ang_grid.w));
  color_img.at<cv::Vec3b>(pans[0]) = cv::Vec3b(0, 255, 0);
  pans = locatePanoramasOnSkeleton(skeleton, pans);
  color_img.at<cv::Vec3b>(pans[0]) = cv::Vec3b(0, 0, 255);
  displayRawImage(color_img, "raw image");

  return 0;
}
