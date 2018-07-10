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

  /*
  rosbag::Bag bag;
  grid_mapping::OccupancyGridPtr grid_ptr = grid.createROSMsg();
  bag.open("data/angle_grid.bag", rosbag::bagmode::Write);
  bag.write("grid", grid_ptr->header.stamp, grid_ptr);
  */

  for (int i = 0; i < grid.layers; ++i) {
    cv::Mat img = createGridImage(grid, i);
    displayImageComplement(img, "grid layers");
  }

  AngleGrid in_grid(Point(-10.0, -10.0), 0.1, 1, 1);
  grid_mapping::OccupancyGridPtr msg = grid.createROSMsg();
  grid_mapping::OccupancyGridConstPtr msg_ptr(new OccupancyGrid(*msg));
  in_grid.insertMap(msg_ptr);
  for (int i = 0; i < in_grid.layers; ++i) {
    cv::Mat img = createGridImage(in_grid, i);
    displayImageComplement(img, "in_grid layers");
  }

  nav_msgs::OccupancyGrid ros_grid = *grid.createROSOGMsg();
  cv::Mat img = createGridImage(ros_grid);
  displayImageComplement(img, "grid img");

  ros_grid = *in_grid.createROSOGMsg();
  img = createGridImage(ros_grid);
  displayImageComplement(img, "in_grid img");

  return 0;
}
