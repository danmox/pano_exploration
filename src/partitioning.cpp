#include "csqmi_planning/partitioning.h"

void locatePanoramasOnSkeleton(const cv::Mat skel, std::vector<cv::Point>& pans)
{
  std::vector<cv::Point> skel_points;
  cv::findNonZero(skel, skel_points);

  for (cv::Point& pan : pans) {
    // check if the panorama is already located on the skeleton
    if (skel.at<unsigned char>(pan) == 1)
      continue;

    // find nearest skeleton cell
    double min_dist = 1e15;
    cv::Point nearest_skel_px = pan;
    for (cv::Point skel_px : skel_points) {
      double dx = pan.x - skel_px.x;
      double dy = pan.y - skel_px.y;
      double dist = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
      if (dist < min_dist) {
        min_dist = dist;
        nearest_skel_px = skel_px;
      }
    }

    // update panorama pixel location
    pan = nearest_skel_px;
  }
}
