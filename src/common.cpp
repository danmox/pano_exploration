#include "csqmi_planning/common.h"

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
