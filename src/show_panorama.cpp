#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat ROSImageToMat(const sensor_msgs::Image::ConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	} catch (cv_bridge::Exception& e) {
		ROS_FATAL("cv_bridge exception: %s", e.what());
		exit(EXIT_FAILURE);
	}

  return cv_ptr->image;
}

int main(int argc, char** argv)
{
  if (argc != 2) {
    ROS_FATAL("Please provide a bagfile");
    exit(EXIT_FAILURE);
  }

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  std::vector<cv::Mat> color_imgs, depth_imgs;
  std::vector<std::string> topics = {"color", "depth"};
  for (auto m : rosbag::View(bag, rosbag::TopicQuery(topics))) {
    if (m.getTopic().compare("color") == 0) {
      cv::Mat rawc = ROSImageToMat(m.instantiate<sensor_msgs::Image>());
      cv::cvtColor(rawc, rawc, CV_BGR2RGB);
      color_imgs.push_back(rawc);
    } else if (m.getTopic().compare("depth") == 0) {
      cv::Mat rawd = ROSImageToMat(m.instantiate<sensor_msgs::Image>());
      double max, min;
      cv::minMaxLoc(rawd, &min, &max);
      cv::Mat scaled_depth;
      rawd.convertTo(scaled_depth, CV_8UC1, 255.0/max);
      cv::applyColorMap(scaled_depth, scaled_depth, cv::COLORMAP_HOT);
      depth_imgs.push_back(scaled_depth);
    }
  }

  if (color_imgs.size() != depth_imgs.size()) {
    ROS_FATAL("The number of color and depth frames do not match");
    exit(EXIT_FAILURE);
  }

  // show color and depth side by side
  cv::namedWindow("Panorama Frames", cv::WINDOW_NORMAL);
  cv::Size csz = color_imgs[0].size();
  cv::Size dsz = depth_imgs[0].size();
  for (int i = 0; i < depth_imgs.size(); ++i) {
    cv::Mat combined = cv::Mat(csz.height, csz.width+dsz.width, CV_8UC3);
    cv::Mat left(combined, cv::Rect(0, 0, csz.width, csz.height));
    color_imgs[i].copyTo(left);
    cv::Mat right(combined, cv::Rect(csz.width, 0, dsz.width, dsz.height));
    depth_imgs[i].copyTo(right);
    cv::imshow("Panorama Frames", combined);
    cv::waitKey(0);
  }

  return 0;
}
