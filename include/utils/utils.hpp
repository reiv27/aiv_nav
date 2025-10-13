#pragma once
#include <cmath>

namespace utils
{
/**
* @brief Normalize angle to be between -M_PI and M_PI
* @param angle Angle to normalize
*/
inline void normalize_angle(double& angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
}

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

} // namespace utils