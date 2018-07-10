#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <grid_mapping/angle_grid.h>
#include <csqmi_planning/csqmi_planning.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char* argv[])
{
  if (argc < 2) {
    ROS_FATAL("Please supply a panorama bag file");
    exit(EXIT_FAILURE);
  }

  ros::init(argc, argv, "skeleton_cloud_pub");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub;
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("skel_pcd", 1); 

  // read panoaram capture location
  rosbag::Bag panbag;
  panbag.open(argv[1], rosbag::bagmode::Read);
  geometry_msgs::TransformStamped pan_pose;
  for (auto m : rosbag::View(panbag, rosbag::TopicQuery("panorama_pose"))) {
    auto msg = m.instantiate<geometry_msgs::TransformStamped>();
    if (msg) {
      pan_pose = *msg;
      break;
    }
  }
  grid_mapping::Point panorama_position(pan_pose.transform.translation.x,
      pan_pose.transform.translation.y);

  // insert panorama
  grid_mapping::AngleGrid ang_grid(grid_mapping::Point(0.0,0.0), 0.1, 1, 1);
  ang_grid.insertPanorama(argv[1]);

  // compute skeleton
  cv::Mat skel;
  computeSkeleton(ang_grid, skel, 2);

  // partition skel
  cv::Point po;
  ang_grid.positionToSubscripts(panorama_position, po.y, po.x);
  std::vector<cv::Point> pan_px(1, po);
  std::vector<std::vector<cv::Point>> regions = partitionSkeleton(skel, pan_px);

  // convert partitioned skeleton to pcl::PointCloud2
  pcl::PointCloud<pcl::PointXYZRGB> skel_cloud;
  for (auto& region : regions) {
    for (auto& px : region) {
      grid_mapping::Point pt = ang_grid.subscriptsToPosition(px.y, px.x);
      pcl::PointXYZRGB pcl_point(0,0,255);
      pcl_point.x = pt.x;
      pcl_point.y = pt.y;
      skel_cloud.points.push_back(pcl_point);
    }
  }
  pan_px = locatePanoramasOnSkeleton(skel, pan_px);
  for (auto pan : pan_px) {
    grid_mapping::Point pan_pt = ang_grid.subscriptsToPosition(pan.y, pan.x);
    pcl::PointXYZRGB pcl_point(255,0,0);
    pcl_point.x = pan_pt.x;
    pcl_point.y = pan_pt.y;
    skel_cloud.points.push_back(pcl_point);
  }
  skel_cloud.width = skel_cloud.points.size();
  skel_cloud.height = 1;
  skel_cloud.is_dense = false;
  //pcl::io::savePCDFileASCII ("skeleton.pcd", skel_cloud);
  //ROS_INFO("Saved pcl to file");

  pcl::PointXYZRGB cpt = skel_cloud.points[rand()%(skel_cloud.points.size())];
  cpt.r = 255;
  cpt.g = 255;
  cpt.b = 0;
  skel_cloud.points.push_back(cpt);
  skel_cloud.width++;
 
  ROS_INFO("Publishing cload at 1 Hz");
  ros::Rate loop(1);
  while(ros::ok()) {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(skel_cloud, ros_cloud);
    ros_cloud.header.frame_id = "world";
    pcl_pub.publish(ros_cloud);
    loop.sleep();
  }

  return 0;
}
