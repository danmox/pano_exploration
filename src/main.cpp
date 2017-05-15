#include "grid_mapping/angle_grid.h"
#include "grid_mapping/common.h"
#include <ros/ros.h>

using namespace grid_mapping;

int main(int argc, char** argv)
{
  if (argc != 2) {
    ROS_FATAL("Please provide a bagfile");
    exit(EXIT_FAILURE);
  }

  AngleGrid grid(Point(0.0, 0.0), 0.1, 1, 1);
  grid.insertPanorama(argv[1]);

  for (int i = 0; i < grid.layers; ++i) {
    cv::Mat img = createGridImage(grid, i);
    displayImageComplement(img, "pan1");
  }

  return 0;
}
