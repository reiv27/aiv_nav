#pragma once
#include <cmath>
#include <vector>
#include <algorithm>

namespace utils
{
/**
* @brief Sign function
* @param x Value to get sign of
* @return Sign of x
*/
inline double sign(double x)
{
  if (x < 0.0) { return -1.0; }
  else { return 1.0; }
}

/**
* @brief Saturation function
* @param x Value to saturate
* @param min_val Minimum value
* @param max_val Maximum value
* @return Saturated value
*/
inline double saturation(double x, double min_val, double max_val)
{
  return std::clamp(x, min_val, max_val);
}

/**
* @brief Distance between two points
* @param x1 First point x
* @param y1 First point y
* @param x2 Second point x
* @param y2 Second point y
* @return Distance between two points
*/
inline double norm2(const std::vector<double>& vec1, const std::vector<double>& vec2)
{
  return std::sqrt(std::pow(vec2[0] - vec1[0], 2) + std::pow(vec2[1] - vec1[1], 2));
}

inline std::vector<double> normilize_vector(const std::vector<double>& v)
{
  std::vector<double> new_vec{v[0] / std::sqrt(v[0] * v[0] + v[1] * v[1]),
                              v[1] / std::sqrt(v[0] * v[0] + v[1] * v[1])};
  return new_vec;
}

inline double dot_product(const std::vector<double>& v1, const std::vector<double>& v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1];
}

inline double angle_between_vectors(const std::vector<double>& v1, 
                                    const std::vector<double>& v2)
{
  const double cross = v1[0] * v2[1] - v1[1] * v2[0];
  const double dot = dot_product(v1, v2);
  return std::abs(std::atan2(cross, dot));
}

inline bool is_point_in_angle(const std::vector<double>& v,
                              const std::vector<double>& p1,
                              const std::vector<double>& p2,
                              const std::vector<double>& r)
{
  const std::vector<double> vp1{p1[0] - v[0], p1[1] - v[1]};
  const std::vector<double> vp2{p2[0] - v[0], p2[1] - v[1]};
  const std::vector<double> vr{r[0] - v[0], r[1] - v[1]};

  const std::vector<double> vp1_norm = normilize_vector(vp1);
  const std::vector<double> vp2_norm = normilize_vector(vp2);
  const std::vector<double> vr_norm = normilize_vector(vr);

  const double angle_p1_p2 = angle_between_vectors(vp1_norm, vp2_norm);
  const double angle_vr_p1 = angle_between_vectors(vr_norm, vp1_norm);
  const double angle_vr_p2 = angle_between_vectors(vr_norm, vp2_norm);

  return (angle_vr_p1 <= angle_p1_p2) && (angle_vr_p2 <= angle_p1_p2);
}

} // namespace utils