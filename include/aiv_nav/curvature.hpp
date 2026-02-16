#pragma once

#include <cstdint>
#include <vector>

namespace aiv_nav::curvature
{
/**
 * @brief Estimate local curvature around closest lidar ray (3-point method).
 *
 * - Finds closest finite range index,
 * - extracts a continuous local segment in +/- half_window rays,
 * - uses three points: left extreme, closest, right extreme,
 * - computes kappa = 2*cross/(L12*L23*L31) (signed curvature).
 * Rejects if points are collinear or any side length is too small.
 *
 * @param kappa_out Output curvature (signed). Set to 0.0 if invalid.
 * @param lidar_ranges Raw lidar ranges (meters).
 * @param lidar_points Cartesian points (x, y) for each ray end.
 * @param r_vis Maximum usable range for curvature (meters).
 * @param half_window Rays to extend left/right from closest ray index.
 * @param break_jump_m Break if neighbor point distance exceeds this value (meters).
 * @return True if curvature is valid, false otherwise.
 */
bool estimate_curvature(double& kappa_out,
                        const std::vector<double>& lidar_ranges,
                        const std::vector<std::vector<double>>& lidar_points,
                        double r_vis,
                        int half_window,
                        double break_jump_m);
}  // namespace aiv_nav::curvature

