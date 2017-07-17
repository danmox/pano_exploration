#include "csqmi_planning/common.h"
#include "csqmi_planning/thinning.hpp"
#include "csqmi_planning/partitioning.h"
#include "csqmi_planning/csqmi.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grid_mapping/angle_grid.h>

#include <algorithm>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  // read OccupancyGrid from saved bag
  Mat img;
  rosbag::Bag bag;
  bag.open("data/pan_grid.bag", rosbag::bagmode::Read);
  grid_mapping::OccupancyGridConstPtr grid;
  for (auto m : rosbag::View(bag)) {
    grid = m.instantiate<grid_mapping::OccupancyGrid>();
    if (grid) {
      occupancyGridToMat(grid, img);
      break;
    }
  }
  grid_mapping::AngleGrid ang_grid(grid);

  // compute skeleton and convert to image
  Mat skeleton, color_img;
  computeSkeleton(img, skeleton);
  cvtColor(255 - img*255.0/100.0, color_img, CV_GRAY2RGB);

  // calculate panorama pixels
  Point o;
  ang_grid.positionToSubscripts(grid_mapping::Point(0.0, 0.0), o.y, o.x);
  vector<Point> pans(1, o);
  pans = locatePanoramasOnSkeleton(skeleton, pans);
  color_img.at<Vec3b>(pans[0]) = Vec3b(0, 0, 255);

  // partition skeleton and show on skeleton image
  vector<vector<Point>> regions = partitionSkeleton(skeleton, pans);
  for (auto& region : regions) {
    Vec3b color(rand()&255, rand()&255, rand()&255);
    for(auto px : region)
      color_img.at<Vec3b>(px) = color;
  }
  displayRawImage(color_img, "skeleton image");

  // convert skeleton region pixels to angle_grid indices
  vector<vector<int>> region_idcs = matPixelsToGridIndices(regions, ang_grid);
  for (auto& idx_vec : region_idcs)
    for (auto idx : idx_vec)
      ang_grid.data[idx] = 10;

  Mat grid_img;
  occupancyGridToMat(ang_grid.createROSMsg(), grid_img);
  displayImageComplement(grid_img, "angle_grid");

  CSQMI objective(DepthCamera(180, 7.0, 0.45), 0.03);
  vector<double> mi = objective.csqmi(ang_grid, regions[0]);
  cout << "max = " << *max_element(mi.begin(), mi.end()) << endl;

  return 0;
}
