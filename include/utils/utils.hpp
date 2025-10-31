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

inline bool is_point_in_angle(const std::vector<double>& vec,
                              const std::vector<double>& point1,
                              const std::vector<double>& point2,
                              const std::vector<double>& r)
{
  // const std::vector<double> vp1{point1[0] - vec[0], point1[1] - vec[1]};
  // const std::vector<double> vp2{point2[0] - vec[0], point2[1] - vec[1]};
  // const std::vector<double> vr{r[0] - vec[0], r[1] - vec[1]};
  // const std::vector<double> vr = r - vec;

  // const double vp1_norm = norm2(vec1);
  // const double vp2_norm = norm2(vec2);
  // const double vr_norm = norm2(vr);

  // const double angle_p1_p2 = std::acos(std::clamp(std::dot(vp1_norm, vp2_norm), -1.0, 1.0));
}
// def is_point_in_angle(V, P1, P2, r_pose):
//     # Calculate vectors
//     VP1 = P1 - V
//     VP2 = P2 - V
//     VR = r_pose - V

//     # Normalize vectors
//     VP1_norm = VP1 / np.linalg.norm(VP1)
//     VP2_norm = VP2 / np.linalg.norm(VP2)
//     VR_norm = VR / np.linalg.norm(VR)

//     # Calculate the angle between P1 and P2 using the dot product
//     angle_P1_P2 = np.arccos(np.clip(np.dot(VP1_norm, VP2_norm), -1.0, 1.0))  # Angle between P1 and P2

//     # Calculate the angle between VR and VP1
//     angle_VR_P1 = np.arccos(np.clip(np.dot(VR_norm, VP1_norm), -1.0, 1.0))

//     # Calculate the angle between VR and VP2
//     angle_VR_P2 = np.arccos(np.clip(np.dot(VR_norm, VP2_norm), -1.0, 1.0))

//     # Check if the angle between VR and both P1 and P2 is less than the angle between P1 and P2
//     return angle_VR_P1 <= angle_P1_P2 and angle_VR_P2 <= angle_P1_P2

} // namespace utils