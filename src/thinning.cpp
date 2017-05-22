#include "csqmi_planning/thinning.hpp"
#include "csqmi_planning/common.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <stack>

std::ostream& operator<<(std::ostream& out, const cv::Vec4i& v) {
  out << "(" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ")";
  return out;
}

void computeSkeleton(cv::Mat& img_in, cv::Mat& img_out)
{
  img_in.convertTo(img_in, CV_8UC1);

  // pad image, invert colors, and convert to binary image
  cv::Mat img;
  cv::copyMakeBorder(img_in, img, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);
  cv::threshold(img, img, 10, 1, cv::THRESH_BINARY_INV); // assumes [0, 100]

  // erode image to cut down on noise
  cv::Mat se = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
  cv::erode(img, img, se, cv::Point(-1,-1), 4);

  auto cond1 = [] (int p2, int p4, int p6, int p8) { 
    return (p2*p4*p6 != 0) || (p4*p6*p8 != 0); };
  auto cond2 = [] (int p2, int p4, int p6, int p8) { 
    return (p2*p4*p8 != 0) || (p2*p6*p8 != 0); };

  cv::Mat prev_img = img.clone();
  while (true) {
    if (subIteration(img, prev_img, cond1) > 0)
      img.copyTo(prev_img);
    else
      break;

    if (subIteration(img, prev_img, cond2) > 0)
      img.copyTo(prev_img);
    else
      break;
  }

  // reduce to 8-connected skeleton
  for (int i = 1; i < img.rows-1; ++i) {
    for (int j = 1; j < img.cols-1; ++j) {
      if (img.at<unsigned char>(i,j) > 0) {
        int p2 = img.at<unsigned char>(i-1, j  );
        int p3 = img.at<unsigned char>(i-1, j+1);
        int p4 = img.at<unsigned char>(i  , j+1);
        int p5 = img.at<unsigned char>(i+1, j+1);
        int p6 = img.at<unsigned char>(i+1, j  );
        int p7 = img.at<unsigned char>(i+1, j-1);
        int p8 = img.at<unsigned char>(i  , j-1);
        int p9 = img.at<unsigned char>(i-1, j-1);

        if (p2 * p4 || p4 * p6 || p6 * p8 || p8 * p2) {
          int b = int( p2 == 0 && p3 == 1 ) + int( p3 == 0 && p4 == 1 ) +
                  int( p4 == 0 && p5 == 1 ) + int( p5 == 0 && p6 == 1 ) +
                  int( p6 == 0 && p7 == 1 ) + int( p7 == 0 && p8 == 1 ) +
                  int( p8 == 0 && p9 == 1 ) + int( p9 == 0 && p2 == 1 );
          if (b < 3)
            img.at<unsigned char>(i,j) = 0;
        }
      }
    }
  }

  // find connected skeleton components
  cv::Mat tmp = img.clone();
  std::vector<std::vector<cv::Point>> cc;
  int largest_comp = 0; // size of largest vector in cc
  for (int i = 1; i < tmp.rows-1; ++i) {
    for (int j = 1; j < tmp.cols-1; ++j) {
      if (tmp.at<unsigned char>(i,j) == 1) {

        std::vector<cv::Point> comp; // cells in current component
        std::stack<cv::Point> next_point; // next cells to search
        next_point.push(cv::Point(j,i));

        // find all connected pixels
        while (!next_point.empty()) {
          cv::Point p = next_point.top();
          comp.push_back(p);
          next_point.pop();
          tmp.at<unsigned char>(p) = 0;
  
          cv::Mat roi = tmp(cv::Rect(p, cv::Size(3,3))-cv::Point(1,1));
          std::vector<cv::Point> neighbors;
          cv::findNonZero(roi, neighbors);

          for (auto n : neighbors) {
            next_point.push(n+p-cv::Point(1,1));
          }
        }

        cc.push_back(comp);
        if (comp.size() > largest_comp)
          largest_comp = comp.size();
      }
    }
  }

  // erase small unconnected skeleton cells
  for (auto comp : cc)
    if (comp.size() != largest_comp) // preserve the main skeleton component
      for (auto point : comp)
        img.at<unsigned char>(point) = 0;

  // store results in output array with pad and threshold removed
  img_out = cv::Mat(img, cv::Rect(1, 1, img.cols-2, img.rows-2)) * 255;
}
