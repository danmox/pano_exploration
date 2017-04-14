#ifndef WAYPOINT_NAVIGATION_NODELET_H
#define WAYPOINT_NAVIGATION_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "panorama/panorama.h"

namespace panorama {

class PanoramaNodelet : public nodelet::Nodelet
{
  private:
    ros::NodeHandle nh, pnh;
    boost::shared_ptr<Panorama> pan;
    
  public:
    virtual void onInit();
};

} // namespace panorama

#endif
