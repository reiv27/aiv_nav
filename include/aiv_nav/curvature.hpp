#pragma once

#include <cstdint>
#include <vector>

namespace aiv_nav::curvature
{
/**
 * @brief Estimate local curvature around closest lidar ray.
 *
 * The function:
 * - finds closest finite range index,
 * - extracts a continuous local segment in +/- half_window rays,
 * - breaks when a scan gap is detected (invalid range or a big point-to-point jump),
 * - fits a circle (least squares) and returns kappa = 1 / R.
 *
 * @param kappa_out Output curvature value (1 / radius). Set to 0.0 if invalid.
 * @param lidar_ranges Raw lidar ranges (meters).
 * @param lidar_points Cartesian points (x, y) for each ray end.
 * @param r_vis Maximum usable range for curvature (meters).
 * @param half_window Rays to extend left/right from closest ray index.
 * @param break_jump_m Break if neighbor point distance exceeds this value (meters).
 * @param max_fit_rms_error_m Reject if circle fit RMS error exceeds this value (meters).
 * @param min_points Minimum points required for circle fit.
 * @return True if curvature is valid, false otherwise.
 */
bool estimate_curvature(double& kappa_out,
                        const std::vector<double>& lidar_ranges,
                        const std::vector<std::vector<double>>& lidar_points,
                        double r_vis,
                        int half_window,
                        double break_jump_m,
                        double max_fit_rms_error_m,
                        uint64_t min_points);
}  // namespace aiv_nav::curvature

