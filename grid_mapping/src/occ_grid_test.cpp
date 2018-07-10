#include "grid_mapping/occ_grid.h"
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

  OccGrid grid(Point(0.0, 0.0), 0.1, 1, 1);
  grid.insertPanorama(argv[1]);

  cv::Mat img = createGridImage(grid);
  displayImageComplement(img, "pan1");

  return 0;
}
