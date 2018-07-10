#ifndef PANORAMA_CONVERT_HPP_
#define PANORAMA_CONVERT_HPP_

#include <tf2_kdl/tf2_kdl.h>

// define tf2::convert(...) interface for geometry_msgs::TransformStamped
// and KDL::Frame types
namespace tf2 {
  inline
  void fromMsg(const geometry_msgs::TransformStamped& msg, KDL::Frame& out)
  {
    out = transformToKDL(msg);
  }
}

#endif
