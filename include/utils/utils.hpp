#pragma once
#include <cmath>
#include <deque>
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
  if (std::abs(x) < 1e-8) {
    return 0.0;
  }
  return x / std::abs(x);
}

/**
* @brief Soft sign function
* @param x Value to get soft sign of
* @param eps Epsilon value
* @return Soft sign of x
*/
inline double soft_sign(double x, double eps=0.1)
{
  if (std::abs(x) < 1e-8) {
    return 0.0;
  }
  return x / (std::abs(x) + eps);
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
* @param vec1 First point (x, y)
* @param vec2 Second point (x, y)
* @return Distance between two points
*/
inline double norm2(const std::vector<double>& vec1, const std::vector<double>& vec2)
{
  const double dx = vec2[0] - vec1[0];
  const double dy = vec2[1] - vec1[1];
  return std::sqrt(dx * dx + dy * dy);
}

inline std::vector<double> normalize_vector(const std::vector<double>& v)
{
  const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1]);
  return {v[0] / norm, v[1] / norm};
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

/**
* @brief Check if a point is in a sector
* @param v Vertex of the sector
* @param p1 First point of the sector
* @param p2 Second point of the sector
* @param r Robot pose (x, y)
* @return True if the point is in the sector, false otherwise
*/
inline bool is_point_in_angle(const std::vector<double>& v,
                              const std::vector<double>& p1,
                              const std::vector<double>& p2,
                              const std::vector<double>& r)
{
  const std::vector<double> vp1{p1[0] - v[0], p1[1] - v[1]};
  const std::vector<double> vp2{p2[0] - v[0], p2[1] - v[1]};
  const std::vector<double> vr{r[0] - v[0], r[1] - v[1]};

  const std::vector<double> vp1_norm = normalize_vector(vp1);
  const std::vector<double> vp2_norm = normalize_vector(vp2);
  const std::vector<double> vr_norm = normalize_vector(vr);

  const double angle_p1_p2 = angle_between_vectors(vp1_norm, vp2_norm);
  const double angle_vr_p1 = angle_between_vectors(vr_norm, vp1_norm);
  const double angle_vr_p2 = angle_between_vectors(vr_norm, vp2_norm);

  return (angle_vr_p1 <= angle_p1_p2) && (angle_vr_p2 <= angle_p1_p2);
}

} // namespace utils