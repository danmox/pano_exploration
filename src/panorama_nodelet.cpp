#include "panorama/panorama_nodelet.h"

namespace panorama {

void PanoramaNodelet::onInit()
{
  nh = getNodeHandle();
  pnh = getPrivateNodeHandle();

  std::string name = "panorama";
  pan.reset(new Panorama(nh, pnh, name));
}

} // namespace panorama

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(panorama::PanoramaNodelet, nodelet::Nodelet);
