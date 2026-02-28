#include "reactive_circumnav/curvature.hpp"

#include <array>
#include <cmath>
#include <limits>

#include "utils/utils.hpp"

namespace
{
constexpr double kCurvatureEps = 1e-6;

int mod_index(int x, int n)
{
  const int r = x % n;
  return (r < 0) ? (r + n) : r;
}

/**
 * Three-point curvature (Menger): kappa = 2*cross / (L12*L23*L31).
 * P1 = left, P2 = center (closest), P3 = right.
 * Rejects if collinear or any side length < eps.
 */
bool curvature_three_point(const std::array<double, 2>& P1,
                          const std::array<double, 2>& P2,
                          const std::array<double, 2>& P3,
                          double& kappa_out)
{
  const double ax = P2[0] - P1[0];
  const double ay = P2[1] - P1[1];
  const double bx = P3[0] - P1[0];
  const double by = P3[1] - P1[1];
  const double cross = ax * by - ay * bx;

  const double L12 = utils::norm2(P2, P1);
  const double L23 = utils::norm2(P3, P2);
  const double L31 = utils::norm2(P3, P1);

  if (L12 < kCurvatureEps || L23 < kCurvatureEps || L31 < kCurvatureEps) {
    return false;
  }
  if (std::abs(cross) < kCurvatureEps) {
    return false;
  }

  const double denom = L12 * L23 * L31;
  if (std::abs(denom) < kCurvatureEps) {
    return false;
  }

  kappa_out = std::abs(2.0 * cross / denom);
  return std::isfinite(kappa_out);
}
}  // namespace

namespace reactive_circumnav::curvature
{
bool estimate_curvature(double& kappa_out,
                        const std::vector<double>& lidar_ranges,
                        const std::vector<std::vector<double>>& lidar_points,
                        double r_vis,
                        int half_window,
                        double break_jump_m)
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
    if (utils::norm2(p, prev_pt) > break_jump_m) {
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
    if (utils::norm2(p, prev_pt) > break_jump_m) {
      break;
    }

    right_pts.push_back(p);
    prev_pt = p;
  }

  // Three points: left extreme, closest, right extreme
  if (left_pts.empty() || right_pts.empty()) {
    return false;
  }

  const std::array<double, 2>& P1 = left_pts.back();
  const std::array<double, 2>& P2 = center_pt;
  const std::array<double, 2>& P3 = right_pts.back();

  return curvature_three_point(P1, P2, P3, kappa_out);
}
}  // namespace reactive_circumnav::curvature

