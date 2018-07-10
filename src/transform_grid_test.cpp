#include "grid_mapping/angle_grid.h"
#include "grid_mapping/common.h"
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <rosbag/bag.h>

using namespace grid_mapping;

int main(int argc, char** argv)
{
  if (argc != 2) {
    ROS_FATAL("Please provide a bagfile");
    exit(EXIT_FAILURE);
  }

  AngleGrid in_grid(Point(0.0, 0.0), 0.1, 1, 1);
  in_grid.insertPanorama(argv[1]);

  /*
  cv::Mat img = createGridImage(*in_grid.createROSOGMsg());
  displayImageComplement(img, "in_grid img");
  */

  geometry_msgs::TransformStamped tfs;
  for (double ang = 0.0; ang <= 2.0*M_PI; ang += M_PI/18) {
    tfs.transform.translation.x = 10.0*cos(ang);
    tfs.transform.translation.y = 10.0*sin(ang);
    tfs.transform.rotation = tf::createQuaternionMsgFromYaw(ang);

    AngleGrid grid(Point(0.0, 0.0), 0.1, 1, 1);
    OccupancyGridConstPtr gcp(new OccupancyGrid(*in_grid.createROSMsg()));
    grid.insertMap(gcp, tfs);

    for (int i = 0; i < grid.layers; ++i) {
      cv::Mat img = createGridImage(grid, i);
      displayImageComplement(img, "grid layers");
    }
  }

  return 0;
}
