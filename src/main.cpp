#include "csqmi_planning/csqmi_planning.h"

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
  vector<InfoPxPair> max_csqmi_pxs;
  for (auto& reg : regions) {
    vector<InfoPxPair> reg_mi = objective.csqmi(ang_grid, reg);
    max_csqmi_pxs.push_back(*max_element(reg_mi.begin(), reg_mi.end(),
        InfoPxPair::compare));
  }

  // sort the max region points and choose the goal point
  std::sort(max_csqmi_pxs.begin(), max_csqmi_pxs.end(), InfoPxPair::compare);
  InfoPxPair goal_pair = max_csqmi_pxs.back();
  grid_mapping::Point goal_pose = ang_grid.subscriptsToPosition(goal_pair.px.y,
      goal_pair.px.x);
  cout << "Goal position is: " << goal_pose << endl;

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
  vector<InfoPxPair> fs_csqmi = objective.csqmi(ang_grid, free_space_pixels);

  // find max CSQMI value and pixel
  InfoPxPair max_pair = *max_element(fs_csqmi.begin(), fs_csqmi.end(), InfoPxPair::compare);

  // create image of CSQMI reward surface
  Mat csqmi_img(ang_grid.h, ang_grid.w, CV_8UC1, Scalar(127));
  for (auto csqmi_pair : fs_csqmi)
    csqmi_img.at<uchar>(csqmi_pair.px) = 255*csqmi_pair.csqmi/(max_pair.csqmi);

  // apply heatmap to CSQMI results image
  Mat color_csqmi_img;
  applyColorMap(csqmi_img, color_csqmi_img, COLORMAP_HOT);

  // overlay grid map on CSQMI results and mark the point of max CSQMI
  color_grid_img.copyTo(color_csqmi_img, 1-free_space_img);
  color_csqmi_img.setTo(Vec3b(0, 255, 0), skeleton);
  color_csqmi_img.at<Vec3b>(max_pair.px) = Vec3b(255, 0, 0);
  cout << "max CSQMI at " << max_pair.px << endl;

  // show the panorama capture location
  pans = locatePanoramasOnSkeleton(skeleton, pans);
  color_csqmi_img.at<Vec3b>(pans[0]) = Vec3b(0, 0, 0); // b,g,r

  // calculate csqmi over skeleton and mark the maximum point of each region in
  // the CSQMI results image
  for (auto csqmi_px : max_csqmi_pxs)
    color_csqmi_img.at<Vec3b>(csqmi_px.px) = Vec3b(255, 0, 0);
  color_csqmi_img.at<Vec3b>(goal_pair.px) = Vec3b(255, 255, 0);

  Mat pixel_density_img;
  pixelDensityGridToMat(px_grid, pixel_density_img);
  displayRawImage(pixel_density_img, "Pixel Density Image");
  displayRawImage(color_csqmi_img, "CSQMI Reward Surface");

  return 0;
}
