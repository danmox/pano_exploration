#ifndef THINNING_HPP
#define THINNING_HPP

#include <opencv2/opencv.hpp>
#include <grid_mapping/angle_grid.h>

void computeSkeleton(grid_mapping::AngleGrid& ang_grid, cv::Mat& img_out,
    int erode_its = 2);

template <typename Func>
int subIteration(cv::Mat img, const cv::Mat prev_img, Func cond)
{
  int num_changes = 0;
  int rows = img.rows;
  int cols = img.cols;

  for (int i = 1; i < rows-1; ++i) {
    for (int j = 1; j < cols-1; ++j) {
      if (img.at<unsigned char>(i,j) == 1) {
        int p2 = prev_img.at<unsigned char>(i-1, j  );
        int p3 = prev_img.at<unsigned char>(i-1, j+1);
        int p4 = prev_img.at<unsigned char>(i  , j+1);
        int p5 = prev_img.at<unsigned char>(i+1, j+1);
        int p6 = prev_img.at<unsigned char>(i+1, j  );
        int p7 = prev_img.at<unsigned char>(i+1, j-1);
        int p8 = prev_img.at<unsigned char>(i  , j-1);
        int p9 = prev_img.at<unsigned char>(i-1, j-1);

        int a = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
        if (a < 2 || a > 6)
          continue;

        int b = int(p2 == 0 && p3 == 1) + int(p3 == 0 && p4 == 1) +
                int(p4 == 0 && p5 == 1) + int(p5 == 0 && p6 == 1) +
                int(p6 == 0 && p7 == 1) + int(p7 == 0 && p8 == 1) +
                int(p8 == 0 && p9 == 1) + int(p9 == 0 && p2 == 1);
        if (b != 1)
          continue;

        if (cond(p2, p4, p6, p8))
          continue;

        img.at<unsigned char>(i, j) = 0;
        ++num_changes;
      }
    }
  }

  return num_changes;
}

#endif
