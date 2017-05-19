#include "csqmi_planning/common.h"
#include "csqmi_planning/thinning.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  Mat img = loadMat("data/map.bin");

  // show image
  Mat skeleton;
  computeSkeleton(img, skeleton);
  img.setTo(50, skeleton);
  displayImageComplement(img, "Map");
  //displayImage(skeleton, "Skeleton");

  return 0;
}
