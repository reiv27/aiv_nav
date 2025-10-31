#pragma once
#include <cmath>
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

} // namespace utils