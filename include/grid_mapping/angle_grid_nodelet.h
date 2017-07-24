#ifndef GRID_MAPPING_ANGLE_GRID_NODELET_H_
#define GRID_MAPPING_ANGLE_GRID_NODELET_H_

#include "grid_mapping/angle_grid.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>

namespace grid_mapping {

class AngleGridNodelet : public nodelet::Nodelet
{
  private:
    ros::NodeHandle pnh, nh;
    ros::Subscriber pan_sub;
    ros::Publisher map_pub;

    std::unique_ptr<AngleGrid> grid_ptr;

  public:
    virtual void onInit();
    void panFileCB(const std_msgs::StringPtr&);
};

} // namespace grid_mapping

#endif
