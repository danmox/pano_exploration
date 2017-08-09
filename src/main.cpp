#include "csqmi_planning/common.h"
#include "csqmi_planning/thinning.hpp"
#include "csqmi_planning/partitioning.h"
#include "csqmi_planning/csqmi.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grid_mapping/angle_grid.h>
#include <grid_mapping/pixel_density_grid.h>
#include <geometry_msgs/TransformStamped.h>

#include <algorithm>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  if (argc < 2) {
    cout << "Please supply a bag with grid_mapping::OccupancyGrid msgs" << endl;
    exit(EXIT_FAILURE);
  }

  // read panorama capture location from saved bag
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  geometry_msgs::TransformStamped pan_pose;
  for (auto m : rosbag::View(bag, rosbag::TopicQuery("panorama_pose"))) {
    auto msg = m.instantiate<geometry_msgs::TransformStamped>();
    if (msg) {
      pan_pose = *msg;
      break;
    }
  }
  grid_mapping::Point panorama_position(pan_pose.transform.translation.x,
      pan_pose.transform.translation.y);
  grid_mapping::AngleGrid ang_grid(grid_mapping::Point(0.0,0.0), 0.1, 1, 1);
  ang_grid.insertPanorama(argv[1]);
  grid_mapping::PixelDensityGrid px_grid(grid_mapping::Point(0.0,0.0),0.1,1,1);
  px_grid.insertPanorama(argv[1]);

  //
  // planning components
  //

  // compute skeleton
  Mat skeleton;
  computeSkeleton(ang_grid, skeleton, 2);

  // partition skeleton
  Point po;
  ang_grid.positionToSubscripts(panorama_position, po.y, po.x);
  vector<Point> pans(1, po);
  vector<vector<Point>> regions = partitionSkeleton(skeleton, pans);

  // find the point of each region with the hightest CSQMI
  CSQMI objective(DepthCamera(180, 6.0, 0.45), 0.03);
  vector<Point> max_csqmi_pts;
  vector<double> region_max_csqmi;
  for (auto& reg : regions) {
    vector<double> reg_mi = objective.csqmi(ang_grid, reg);
    int idx = max_element(reg_mi.begin(), reg_mi.end()) - reg_mi.begin();
    max_csqmi_pts.push_back(reg[idx]);
    region_max_csqmi.push_back(reg_mi[idx]);
  }

  // choose the goal point
  auto goal_it = max_element(region_max_csqmi.begin(), region_max_csqmi.end());
  Point goal_px = max_csqmi_pts[goal_it - region_max_csqmi.begin()];
  grid_mapping::Point goal = ang_grid.subscriptsToPosition(goal_px.y, goal_px.x);
  cout << "Goal position is: " << goal << endl;

  //
  // visualization components
  //

  // create BW image of skeleton and color image of occupancy grid
  Mat grid_img;
  occupancyGridToMat(ang_grid, grid_img);
  Mat color_grid_img;
  displayRawImage(skeleton*255, "skeleton image");
  cvtColor(255 - grid_img*255.0/100.0, color_grid_img, CV_GRAY2RGB);

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
  pans = locatePanoramasOnSkeleton(skeleton, pans);
  color_csqmi_img.at<Vec3b>(pans[0]) = Vec3b(0, 0, 0); // b,g,r

  // calculate csqmi over skeleton and mark the maximum point of each region in
  // the CSQMI results image
  for (auto& max_pt : max_csqmi_pts)
    color_csqmi_img.at<Vec3b>(max_pt) = Vec3b(255, 0, 0);
  color_csqmi_img.at<Vec3b>(goal_px) = Vec3b(255, 255, 0);

  Mat pixel_density_img;
  pixelDensityGridToMat(px_grid, pixel_density_img);
  displayRawImage(pixel_density_img, "Pixel Density Image");
  displayRawImage(color_csqmi_img, "CSQMI Reward Surface");

  return 0;
}
