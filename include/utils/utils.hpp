#pragma once
#include <cmath>

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

} // namespace utils