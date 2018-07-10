#include "grid_mapping/angle_grid_nodelet.h"

#include <grid_mapping/OccupancyGrid.h>

namespace grid_mapping {

void AngleGridNodelet::onInit()
{
  nh = getNodeHandle();
  pnh = getPrivateNodeHandle();

  double origin_x, origin_y, res;
  int bins;
  if (!pnh.getParam("origin_x", origin_x) ||
      !pnh.getParam("origin_y", origin_y) ||
      !pnh.getParam("resolution", res) ||
      !pnh.getParam("bins", bins)) {
    ROS_FATAL("AngleGridNodelet::onInit(): unable to read ros parameters");
    exit(EXIT_FAILURE);
  }
  grid_ptr.reset(new AngleGrid(Point(origin_x, origin_y), res, 1, 1, bins));

  pan_sub = nh.subscribe("pan_files", 2, &AngleGridNodelet::panFileCB, this);
  map_pub = nh.advertise<OccupancyGrid>("angle_grids", 2);
}

void AngleGridNodelet::panFileCB(const std_msgs::StringPtr& file)
{
  grid_ptr->insertPanorama(file->data);
  map_pub.publish(grid_ptr->createROSMsg());
}

} // namespace grid_mapping

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grid_mapping::AngleGridNodelet, nodelet::Nodelet);
