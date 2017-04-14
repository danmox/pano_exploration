#ifndef PANORAMA_NODELET_H_
#define PANORAMA_NODELET_H_

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
