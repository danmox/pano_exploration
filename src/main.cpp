#include "grid_mapping/angle_grid.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <algorithm>
#include <vector>
#include <iterator>
#include <cmath>
#include <iostream>

using namespace grid_mapping;

cv::Mat createGridImage(const OccGrid& grid)
{
  std::vector<int> map_data;
  std::transform(grid.data.begin(), grid.data.end(), 
      std::back_inserter(map_data), 
      [](double a){ return static_cast<int>(255*(1.0/(1.0+exp(-a)))) ;});
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}

cv::Mat createGridImage(const AngleGrid& grid, const unsigned int layer=0)
{
  if (layer >= grid.layers) {
    ROS_FATAL("createAngleGridImage(...): invalid layer argument");
    exit(EXIT_FAILURE);
  }
  const int layer_size = grid.w*grid.h;
  std::vector<int> map_data;
  std::transform(grid.data.begin() + layer*layer_size, 
      grid.data.begin() + (layer+1)*layer_size, 
      std::back_inserter(map_data), 
      [](double a){ return static_cast<int>(255*(1.0/(1.0+exp(-a)))) ;});
  cv::Mat img = cv::Mat(map_data).reshape(0, grid.h);
  img.convertTo(img, CV_8UC1);
  cv::flip(img, img, 0);
  return img;
}

void displayImage(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, img);
  cv::waitKey(0);
}

void displayImageComplement(const cv::Mat& img, const std::string name)
{
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::imshow(name, 255-img);
  cv::waitKey(0);
}

int main(int argc, char** argv)
{
  if (argc != 2) {
    ROS_FATAL("Please provide a bagfile");
    exit(EXIT_FAILURE);
  }
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  std::vector<sensor_msgs::LaserScan> scans;
  std::vector<geometry_msgs::Pose2D> poses;
  for (auto m : rosbag::View(bag)) {
    sensor_msgs::LaserScanConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
    if (s)
      scans.push_back(*s);
    nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
    if (odom) {
      geometry_msgs::Pose2D pose;
      pose.x = odom->pose.pose.position.x;
      pose.y = odom->pose.pose.position.y;
      pose.theta = tf::getYaw(odom->pose.pose.orientation);
      poses.push_back(pose);
    }
  }

  if (scans.size() != poses.size()) {
    ROS_FATAL("Unequal number of scans (%ld) and poses (%ld)", scans.size(),
        poses.size());
    exit(EXIT_FAILURE);
  }

  Point origin(poses[0].x, poses[0].y);
  AngleGrid grid(origin, 0.1, 1, 1);
  for (int i = 0; i < scans.size(); ++i) {
    sensor_msgs::LaserScanConstPtr scan(new sensor_msgs::LaserScan(scans[i]));
    geometry_msgs::Pose2DConstPtr pose(new geometry_msgs::Pose2D(poses[i]));
    grid.insertScan(scan, pose);
    //displayImageComplement(createGridImage(grid), "grid");
  }

  for (int i = 0; i < grid.layers; ++i)
    displayImageComplement(createGridImage(grid,i), "angle grid");

  return 0;
}
