#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include "csqmi_planning/common.h"
#include "csqmi_planning/thinning.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  const char* home = getenv("HOME");
  ifstream f(string(home) + "/catkin_ws/src/csqmi_planning/data/map.bin", 
      ios::in | ios::binary | ios::ate);
  if (!f.is_open()) {
    cout << "Error opening map.bin" << endl;
    exit(-1);
  }

  size_t sz = f.tellg();
  vector<char> map_data(sz);
  f.seekg(0, ios::beg);
  f.read(reinterpret_cast<char*>(map_data.data()), sz);

  if (map_data.size() != 258*292) {
    cout << "Input data and Mat dimensions do not agree" << endl;
    exit(-1);
  }

  Mat img = Mat(292, 258, CV_8UC1, map_data.data()).t();

  // show image
  Mat skeleton;
  computeSkeleton(img, skeleton);
  img.setTo(50, skeleton);
  displayImageComplement(img, "Map");
  //displayImage(skeleton, "Skeleton");

  return 0;
}
