#include "aiv_nav/curvature.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
int mod_index(int x, int n)
{
  const int r = x % n;
  return (r < 0) ? (r + n) : r;
}

double dist_xy(const std::array<double, 2>& a, const std::array<double, 2>& b)
{
  const double dx = b[0] - a[0];
  const double dy = b[1] - a[1];
  return std::sqrt(dx * dx + dy * dy);
}

bool solve_3x3(std::array<std::array<double, 3>, 3> A,
               std::array<double, 3> b,
               std::array<double, 3>& x)
{
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    double pivot_abs = std::abs(A[col][col]);

    for (int row = col + 1; row < 3; ++row) {
      const double v = std::abs(A[row][col]);
      if (v > pivot_abs) {
        pivot_abs = v;
        pivot = row;
      }
    }

    if (pivot_abs < 1e-12) {
      return false;
    }

    if (pivot != col) {
      std::swap(A[pivot], A[col]);
      std::swap(b[pivot], b[col]);
    }

    const double inv = 1.0 / A[col][col];
    for (int k = col; k < 3; ++k) {
      A[col][k] *= inv;
    }
    b[col] *= inv;

    for (int row = 0; row < 3; ++row) {
      if (row == col) {
        continue;
      }

      const double factor = A[row][col];
      for (int k = col; k < 3; ++k) {
        A[row][k] -= factor * A[col][k];
      }
      b[row] -= factor * b[col];
    }
  }

  x[0] = b[0];
  x[1] = b[1];
  x[2] = b[2];
  return true;
}

bool fit_circle_least_squares(const std::vector<std::array<double, 2>>& pts,
                              std::array<double, 2>& center,
                              double& radius,
                              double& rms_error)
{
  if (pts.size() < 3) {
    return false;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto& p : pts) {
    mean_x += p[0];
    mean_y += p[1];
  }

  const double inv_n = 1.0 / static_cast<double>(pts.size());
  mean_x *= inv_n;
  mean_y *= inv_n;

  double Sxx = 0.0;
  double Syy = 0.0;
  double Sxy = 0.0;
  double Sx = 0.0;
  double Sy = 0.0;
  double SbX = 0.0;
  double SbY = 0.0;
  double Sb1 = 0.0;

  for (const auto& p : pts) {
    const double x = p[0] - mean_x;
    const double y = p[1] - mean_y;
    const double bb = -(x * x + y * y);

    Sxx += x * x;
    Syy += y * y;
    Sxy += x * y;
    Sx += x;
    Sy += y;

    SbX += x * bb;
    SbY += y * bb;
    Sb1 += bb;
  }

  const double N = static_cast<double>(pts.size());
  const std::array<std::array<double, 3>, 3> A{{
    {{Sxx, Sxy, Sx}},
    {{Sxy, Syy, Sy}},
    {{Sx,  Sy,  N}}
  }};
  const std::array<double, 3> b{{SbX, SbY, Sb1}};

  std::array<double, 3> sol{{0.0, 0.0, 0.0}};
  if (!solve_3x3(A, b, sol)) {
    return false;
  }

  const double D = sol[0];
  const double E = sol[1];
  const double F = sol[2];

  const double cx_s = -0.5 * D;
  const double cy_s = -0.5 * E;
  const double r2 = cx_s * cx_s + cy_s * cy_s - F;
  if (!(r2 > 0.0) || !std::isfinite(r2)) {
    return false;
  }

  radius = std::sqrt(r2);
  if (!std::isfinite(radius) || (radius < 1e-6)) {
    return false;
  }

  center[0] = cx_s + mean_x;
  center[1] = cy_s + mean_y;

  double sum_sq = 0.0;
  for (const auto& p : pts) {
    const double dx = p[0] - center[0];
    const double dy = p[1] - center[1];
    const double ri = std::sqrt(dx * dx + dy * dy);
    const double e = ri - radius;
    sum_sq += e * e;
  }

  rms_error = std::sqrt(sum_sq / N);
  return std::isfinite(rms_error);
}
}  // namespace

namespace aiv_nav::curvature
{
bool estimate_curvature(double& kappa_out,
                        const std::vector<double>& lidar_ranges,
                        const std::vector<std::vector<double>>& lidar_points,
                        double r_vis,
                        int half_window,
                        double break_jump_m,
                        double max_fit_rms_error_m,
                        uint64_t min_points)
{
  kappa_out = 0.0;

  if (lidar_ranges.empty() || lidar_points.empty()) {
    return false;
  }
  if (lidar_ranges.size() != lidar_points.size()) {
    return false;
  }

  const int n = static_cast<int>(lidar_ranges.size());
  if (n < 3) {
    return false;
  }
  if (half_window <= 0) {
    return false;
  }

  double min_range = std::numeric_limits<double>::infinity();
  int center_idx = -1;
  for (int i = 0; i < n; ++i) {
    const double r = lidar_ranges[static_cast<size_t>(i)];
    if (!std::isfinite(r)) {
      continue;
    }
    if (r > r_vis) {
      continue;
    }
    if (r < min_range) {
      min_range = r;
      center_idx = i;
    }
  }
  if (center_idx < 0) {
    return false;
  }

  auto get_xy = [&lidar_points](int idx, std::array<double, 2>& p_out) -> bool {
    const auto& v = lidar_points[static_cast<size_t>(idx)];
    if (v.size() < 2) {
      return false;
    }
    p_out[0] = v[0];
    p_out[1] = v[1];
    return std::isfinite(p_out[0]) && std::isfinite(p_out[1]);
  };

  std::vector<std::array<double, 2>> left_pts;
  std::vector<std::array<double, 2>> right_pts;
  left_pts.reserve(static_cast<size_t>(half_window));
  right_pts.reserve(static_cast<size_t>(half_window));

  std::array<double, 2> center_pt{{0.0, 0.0}};
  if (!get_xy(center_idx, center_pt)) {
    return false;
  }

  std::array<double, 2> prev_pt = center_pt;
  for (int step = 1; step <= half_window; ++step) {
    const int idx = mod_index(center_idx - step, n);
    const double r = lidar_ranges[static_cast<size_t>(idx)];
    if (!std::isfinite(r) || (r > r_vis)) {
      break;
    }

    std::array<double, 2> p{{0.0, 0.0}};
    if (!get_xy(idx, p)) {
      break;
    }
    if (dist_xy(p, prev_pt) > break_jump_m) {
      break;
    }

    left_pts.push_back(p);
    prev_pt = p;
  }

  prev_pt = center_pt;
  for (int step = 1; step <= half_window; ++step) {
    const int idx = mod_index(center_idx + step, n);
    const double r = lidar_ranges[static_cast<size_t>(idx)];
    if (!std::isfinite(r) || (r > r_vis)) {
      break;
    }

    std::array<double, 2> p{{0.0, 0.0}};
    if (!get_xy(idx, p)) {
      break;
    }
    if (dist_xy(p, prev_pt) > break_jump_m) {
      break;
    }

    right_pts.push_back(p);
    prev_pt = p;
  }

  std::vector<std::array<double, 2>> local_pts;
  local_pts.reserve(left_pts.size() + 1 + right_pts.size());

  for (int i = static_cast<int>(left_pts.size()) - 1; i >= 0; --i) {
    local_pts.push_back(left_pts[static_cast<size_t>(i)]);
  }
  local_pts.push_back(center_pt);
  for (const auto& p : right_pts) {
    local_pts.push_back(p);
  }

  if (local_pts.size() < static_cast<size_t>(min_points)) {
    return false;
  }

  std::array<double, 2> c{{0.0, 0.0}};
  double R = 0.0;
  double rms = 0.0;
  if (!fit_circle_least_squares(local_pts, c, R, rms)) {
    return false;
  }

  if (rms > max_fit_rms_error_m) {
    return false;
  }
  if (!std::isfinite(R) || (R < 1e-6)) {
    return false;
  }

  if (R > 1e6) {
    kappa_out = 0.0;
    return true;
  }

  kappa_out = 1.0 / R;
  return std::isfinite(kappa_out);
}
}  // namespace aiv_nav::curvature

