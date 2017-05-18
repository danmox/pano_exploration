#include "grid_mapping/angle_grid.h"
#include "grid_mapping/common.h"

#include <ros/ros.h>
#include <rosbag/bag.h>

using namespace grid_mapping;

int main(int argc, char** argv)
{
  if (argc != 2) {
    ROS_FATAL("Please provide a bagfile");
    exit(EXIT_FAILURE);
  }

  AngleGrid grid(Point(0.0, 0.0), 0.1, 1, 1);
  grid.insertPanorama(argv[1]);

  rosbag::Bag bag;
  grid_mapping::OccupancyGridPtr grid_ptr = grid.createROSMsg();
  bag.open("data/pan_grid.bag", rosbag::bagmode::Write);
  bag.write("grid", grid_ptr->header.stamp, grid_ptr);

  for (int i = 0; i < grid.layers; ++i) {
    cv::Mat img = createGridImage(grid, i);
    displayImageComplement(img, "pan1");
  }

  return 0;
}
