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
  for (auto m : rosbag::View(bag)) {
    grid_mapping::OccupancyGridConstPtr grid;
    grid = m.instantiate<grid_mapping::OccupancyGrid>();
    if (grid) {
      occupancyGridToMat(grid, img);
      break;
    }
  }

  // show image
  cv::Mat skeleton;
  computeSkeleton(img, skeleton);
  img.setTo(50, skeleton);
  displayImageComplement(img, "Map");
  displayImage(skeleton, "Skeleton");

  return 0;
}
