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
  Mat grid_img;
  rosbag::Bag bag;
  bag.open("data/pan_grid.bag", rosbag::bagmode::Read);
  grid_mapping::OccupancyGridConstPtr grid;
  for (auto m : rosbag::View(bag)) {
    grid = m.instantiate<grid_mapping::OccupancyGrid>();
    if (grid)
      break;
  }

  // create AngleGrid and opencv image from saved grid_mapping::OccupancyGrid
  grid_mapping::AngleGrid ang_grid(grid);
  occupancyGridToMat(ang_grid, grid_img);

  // compute skeleton and convert to image
  Mat skeleton, color_grid_img;
  computeSkeleton(grid_img, skeleton);
  cvtColor(255 - grid_img*255.0/100.0, color_grid_img, CV_GRAY2RGB);

  // calculate the pixel of the panorama capture location
  Point o;
  ang_grid.positionToSubscripts(grid_mapping::Point(0.0, 0.0), o.y, o.x);
  vector<Point> pans(1, o);
  pans = locatePanoramasOnSkeleton(skeleton, pans);

  // partition skeleton
  vector<vector<Point>> regions = partitionSkeleton(skeleton, pans);
  //displayRawImage(skeleton_img, "skeleton image");

  // initialize CSQMI objective function
  CSQMI objective(DepthCamera(180, 7.0, 0.45), 0.03);

  // find all free space pixels
  Mat free_space_img;
  threshold(grid_img, free_space_img, 5, 1, THRESH_BINARY_INV);
  vector<Point> free_space_pixels;
  findNonZero(free_space_img, free_space_pixels);

  // compute CSQMI for all free space pixels
  cout << "computing csqmi for " << free_space_pixels.size() <<" points"<< endl;
  vector<double> mi = objective.csqmi(ang_grid, free_space_pixels);

  // find max CSQMI value and pixel
  auto max_it = max_element(mi.begin(), mi.end());
  Point max_point = free_space_pixels[max_it - mi.begin()];

  // create image of CSQMI reward surface
  Mat csqmi_img(ang_grid.h, ang_grid.w, CV_8UC1, Scalar(127));
  for (int i = 0; i < free_space_pixels.size(); ++i)
    csqmi_img.at<uchar>(free_space_pixels[i]) = 255*mi[i]/(*max_it); 

  // apply heatmap to CSQMI results image
  Mat color_csqmi_img;
  applyColorMap(csqmi_img, color_csqmi_img, COLORMAP_HOT);

  // overlay grid map on CSQMI results and mark the point of max CSQMI
  color_grid_img.copyTo(color_csqmi_img, 1-free_space_img);
  color_csqmi_img.setTo(Vec3b(0, 255, 0), skeleton);
  color_csqmi_img.at<Vec3b>(max_point) = Vec3b(255, 0, 0);
  cout << "max CSQMI at " << max_point << endl;

  // show the panorama capture location
  color_csqmi_img.at<Vec3b>(pans[0]) = Vec3b(0, 0, 0); // b,g,r

  // calculate csqmi over skeleton and mark the maximum point of each region in
  // the CSQMI results image
  for (auto& reg : regions) {
    vector<double> reg_mi = objective.csqmi(ang_grid, reg);
    int idx = max_element(reg_mi.begin(), reg_mi.end()) - reg_mi.begin();
    cout << "max region CSQMI at " << reg[idx] << endl;
    color_csqmi_img.at<Vec3b>(reg[idx]) = Vec3b(255, 0, 0);
  }

  displayRawImage(color_csqmi_img, "CSQMI Reward Surface");

  return 0;
}
