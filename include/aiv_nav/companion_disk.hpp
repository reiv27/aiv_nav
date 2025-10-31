#pragma once

#include <vector>
#include <cstdint>

class CompanionDisk
{
public:
  CompanionDisk(double R, uint64_t resolution, uint64_t window_size);

  /**
   * @brief Get disk pose
   * @return Disk pose (x, y)
   */
  const std::vector<double>& get_pose() const;
  
  /**
   * @brief Get rays length
   * @return Rays length
   */
  const std::vector<double>& get_rays_length() const;

  /**
   * @brief Update disk pose
   * @param robot_state Robot state (x, y, theta)
   * @param lidar_closest_point Lidar closest point
   * @param distance Distance from closest point to disk pose
   */
  void update_pose(const std::vector<double>& robot_state,
                   const std::vector<double>& lidar_closest_point, double distance);

  /**
   * @brief Update rays length
   * @param lidar_points Lidar points
   */
  void update_rays_length(const std::vector<std::vector<double>>& lidar_points);

  /**
   * @brief Get disk min length
   * @return Disk min length
   */
  double get_min_ray_length() const;
  
private:
  double R_;
  const uint64_t resolution_;
  const uint64_t window_size_;

  std::vector<double> pose_{0.0, 0.0};
  std::vector<double> rays_length_;
  double min_ray_length_{0.0};
  int min_arg_{0};
};