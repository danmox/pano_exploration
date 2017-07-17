#include <csqmi_planning/csqmi.h>
#include <ros/ros.h>

CSQMI::CSQMI(const DepthCamera depth_camera, const double stdev_) :
  sensor(depth_camera),
  N(sqrt(2.0)*stdev_),
  independence_threshold(0.1)
{
}

// calculate the probability of the beam reaching a cell in c
std::vector<double> computePEs(const std::vector<double> &o)
{
  size_t pew_size = o.size() + 1;

  std::vector<double> pe(pew_size, 0.0); 
  double product = 1.0;
  pe[1] = o[0];
  for (int j = 2; j < pew_size; ++j) {
    product *= (1.0 - o[j-2]);
    pe[j] = product*o[j-1];
  }
  pe[0] = product * (1.0 - o.back());

  return pe;
}

std::vector<double> computeWeights(const std::vector<double> &o, 
    const std::vector<double> &pe)
{
  size_t pew_size = o.size()+1;

  std::vector<double> w(pew_size, 0.0); 
  double product = 1;
  w[0] = pow(pe[0], 2);
  w.back() = pow(pe.back(), 2);
  for (int k = pew_size-2; k > 0; --k) {
    product *= pow(o[k], 2) + pow(1.0 - o[k], 2);
    w[k] = pow(pe[k], 2) * product;
  }

  return w;
}

// determine if beam can reasonably be considered independent
bool CSQMI::isIndependent(const std::vector<double> &pe, 
    const std::vector<int>& cells, hit_map& any_hit)
{
  std::unordered_map<int, double>::iterator it, end = any_hit.end();
  for (int i = 0; i < cells.size(); ++i)
    if ((it = any_hit.find(cells[i])) != end)
      if (std::min(pe[i], it->second) > independence_threshold)
        return false;
  return true;
}

void CSQMI::updateAnyHit(const std::vector<double> &pe, 
    const std::vector<int>& cells, hit_map& any_hit)
{
  std::unordered_map<int, double>::iterator it;
  for (int i = 0; i < cells.size(); ++i) {
    it = any_hit.find(cells[i]);
    if (it != any_hit.end())
      it->second += pe[i];
    else
      any_hit.emplace(cells[i], pe[i]);
  }
}

// CSQMI algorithm for a single beam
double CSQMI::beam_csqmi(const std::vector<double> &o, 
    const std::vector<int> &cells, hit_map& any_hit)
{
  // calculate probability values p(e_i) 
  std::vector<double> pe = computePEs(o);

  // determine if the beam is reasonably independent
  if (!isIndependent(pe, cells, any_hit))
    return 0.0;
  updateAnyHit(pe, cells, any_hit);

  // calculate weights w_l
  std::vector<double> w = computeWeights(o, pe);

  // term 1
  double sum1 = 0.0;
  for (double w_i : w)
    sum1 += w_i;
  double mi = log2(N(0.0)*sum1);

  // constant for terms 2 & 3
  double C2 = 1.0;
  for (double o_i : o)
    C2 *= pow(o_i, 2) + pow(1.0 - o_i, 2);

  // terms 2 & 3
  double cell_increment = sensor.max_range / ((double)(o.size()-1));
  double sum2 = 0.0, sum3 = 0.0;
  int B = ((int)(4.0*N.getStdev() / cell_increment)) + 1;
  size_t pew_size = pe.size();
  for (int j = 0; j < pew_size; ++j) {

    // compute distance along beam and enforce the sensor's min range
    double uj = j * cell_increment;
    if (uj < sensor.min_range) uj = 0.0;
    
    // only consider B cells on either side of c_j since Gaussians fall 
    // off dramatically a few standard deviations away from the mean
    int start_index = j - B;
    if (start_index < 0) start_index = 0;
    int end_index = j + B + 1; // one past the last desired cell
    if (end_index > pew_size) end_index = pew_size;

    for (int l = start_index; l < end_index; ++l) {

      // compute distance along beam and enforce the sensor's min range
      double ul = l * cell_increment;
      if ( ul < sensor.min_range ) ul = 0.0;

      double C3 = N(ul - uj);
      sum2 += pe[j] * pe[l] * C3;
      sum3 += pe[j] * w[l] * C3;
    }
  }
  mi += log2(C2*sum2);
  mi -= 2.0*log2(sum3);

  return mi;
}

double CSQMI::csqmi(const grid_mapping::AngleGrid& grid, 
    const grid_mapping::Point& ray_start)
{
  double mi = 0.0;

  // initialize hit_map and reserve enough space for all beams
  hit_map any_hit;
  any_hit.reserve(M_PI*pow(sensor.max_range, 2)/grid.resolution);

  // compute CSQMI for each beam
  double angle_increment = (1.0 / (double)(sensor.resolution)) * 2.0 * M_PI;
  int layer = grid.w*grid.h;
  for (int index = 0; index < sensor.resolution; ++index) {

    // determine ray endpoints
    double angle = ((double)(index)) * angle_increment;
    grid_mapping::Point ray_end = ray_start + 
      sensor.max_range*grid_mapping::Point(cos(angle), sin(angle));

    // find cells along the beam
    std::vector<int> cells = grid.rayCast(ray_start, ray_end);
    int c_size = cells.size();
    if (c_size == 0) {
      ROS_FATAL("CSQMI::csqmi(...): rayCast failed, cells.size() = 0");
      exit(EXIT_FAILURE);
    }

    // determine true beam length
    grid_mapping::Point dP = ray_end - ray_start;
    int true_beam_length = round(dP.abs().max() / grid.resolution);

    // determine cell values along beam
    std::vector<double> o;
    o.reserve(true_beam_length);
    int angle_index = grid.angleIndex(angle);
    for (int c : cells)
      o.push_back(grid.cellProb(c + angle_index*layer));

    // add "unknown" cells for beams that reach out of bounds
    for (int j = c_size; j < true_beam_length; ++j)
      o.push_back(0.5);

    // aggregate CSQMI
    mi += beam_csqmi(o, cells, any_hit);
  }
}

std::vector<double> CSQMI::csqmi(const grid_mapping::AngleGrid& grid, 
    const std::vector<cv::Point>& pixels)
{
  std::vector<double> mi;
  mi.reserve(pixels.size());
  for (auto px : pixels)
    mi.push_back(csqmi(grid, grid.subscriptsToPosition(px.y, px.x)));

  return mi;
}
