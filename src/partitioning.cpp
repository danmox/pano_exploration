#include "csqmi_planning/partitioning.h"
#include "csqmi_planning/common.h"

#include <stack>

using namespace std;
using namespace cv;

vector<Point> locatePanoramasOnSkeleton(const Mat skel, vector<Point> pans)
{
  vector<Point> skel_points;
  findNonZero(skel, skel_points);

  for (Point& pan : pans) {
    // check if the panorama is already located on the skeleton
    if (skel.at<unsigned char>(pan) == 1)
      continue;

    // find nearest skeleton cell
    double min_dist = 1e15;
    Point nearest_skel_px = pan;
    for (Point skel_px : skel_points) {
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

  return pans;
}

vector<vector<Point>> partitionSkeleton(const Mat skel_in,
    const vector<Point> pans)
{
  // remove panorama pixels from skeleton
  vector<Point> pan_pixels = locatePanoramasOnSkeleton(skel_in, pans);
  Mat skel = skel_in.clone();
  for (Point pan : pan_pixels)
    skel.at<unsigned char>(pan) = 0;

  // find connected regions
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(skel, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

  /*
  Mat dst = Mat::zeros(skel.rows, skel.cols, CV_8UC3);
  for (int idx = 0 ; idx >= 0; idx = hierarchy[idx][0]) {
      Scalar color(rand()&255, rand()&255, rand()&255);
      drawContours(dst, contours, idx, color, FILLED, 8, hierarchy);
  }
  displayRawImage(dst, "components");
  */

  return contours;
}

vector<vector<int>> matPixelsToGridIndices(const vector<vector<Point>>& regions,
    const grid_mapping::AngleGrid& ang_grid)
{
  vector<vector<int>> region_grid_indices;
  for (auto& region : regions) {
    vector<int> indices;
    indices.reserve(region.size());
    for (auto px : region) {
      int idx = ang_grid.subscriptsToIndex(px.y, px.x);
      indices.push_back(idx);
    }
    region_grid_indices.push_back(indices);
  }

  return region_grid_indices;
}
