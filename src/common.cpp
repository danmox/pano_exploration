#include "csqmi_planning/common.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace cv;

void displayImageComplement(const Mat& img, const string name)
{
  namedWindow(name, WINDOW_NORMAL);
  double min, max;
  minMaxLoc(img, &min, &max);
  imshow(name, 255 - img*(255/max));
  waitKey(0);
}

void displayImage(const Mat& img, const string name)
{
  namedWindow(name, WINDOW_NORMAL);
  double min, max;
  minMaxLoc(img, &min, &max);
  imshow(name, img*(255/max));
  waitKey(0);
}

Mat loadMat(string file)
{
  ifstream f(file, ios::in | ios::binary | ios::ate);
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

  return Mat(292, 258, CV_8UC1, map_data.data()).t();
}
