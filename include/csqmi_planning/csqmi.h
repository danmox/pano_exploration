#ifndef CSQMI_PLANNING_CSQMI_H_
#define CSQMI_PLANNING_CSQMI_H_

#include <grid_mapping/angle_grid.h>
#include <opencv2/core/types.hpp>
#include <unordered_map>
#include <vector>
#include <cmath>

struct DepthCamera
{
  int resolution;
  double max_range;
  double min_range;

  DepthCamera(int res_, double max_, double min_) :
      resolution(res_), max_range(max_), min_range(min_)
  {
  }
};

class Gaussian
{
  private:
    double stdev, a, b;

  public:
    Gaussian(double stdev_) : 
      stdev(stdev_) 
    {
      a = 1.0 / (stdev_ * sqrt(2.0*M_PI));
      b = -1.0 / (2.0 * pow(stdev_, 2.0));
    }

    double operator()(const double mean, const double x) const
    {
      return a*exp(b*pow(x-mean, 2));
    }

    double operator()(const double u) const
    {
      return a*exp(b*pow(u, 2));
    }

    double getStdev() const { return stdev; }
};

typedef std::unordered_map<int, double> hit_map;

class CSQMI
{
  private:
    const DepthCamera sensor;
    const Gaussian N;
    const double independence_threshold;

    bool isIndependent(const std::vector<double>&, const std::vector<int>&, 
        hit_map&);
    void updateAnyHit(const std::vector<double>&, const std::vector<int>&,
        hit_map&);

  public:
    CSQMI(const DepthCamera, const double);

    double beam_csqmi(const std::vector<double>&, const std::vector<int>&,
        hit_map&);
    double csqmi(const grid_mapping::AngleGrid&, const grid_mapping::Point&);
    std::vector<double> csqmi(const grid_mapping::AngleGrid&, 
        const std::vector<cv::Point>&);
};

#endif
