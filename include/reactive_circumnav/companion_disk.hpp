#ifndef REACTIVE_CIRCUMNAV_COMPANION_DISK_HPP
#define REACTIVE_CIRCUMNAV_COMPANION_DISK_HPP

#include <cstdint>
#include <vector>

class CompanionDisk
{
public:
  CompanionDisk();
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
   * @param lidar_data Lidar data
   */
  void update_rays_length(const std::vector<std::vector<double>>& lidar_points,
                          const std::vector<double>& lidar_data);

  /**
   * @brief Get disk min length
   * @return Disk min length
   */
  double get_min_ray_length() const;

  /**
   * @brief Get disk min arg
   * @return Disk min arg
   */
  uint64_t get_min_arg() const;

private:
  double R_;
  uint64_t resolution_;
  uint64_t window_size_;

  std::vector<double> pose_{0.0, 0.0};
  std::vector<double> rays_length_;
  double min_ray_length_{0.0};
  uint64_t min_arg_{0};
};

#endif  // REACTIVE_CIRCUMNAV_COMPANION_DISK_HPP
