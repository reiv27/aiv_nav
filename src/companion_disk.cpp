#include "aiv_nav/companion_disk.hpp"
#include "utils/utils.hpp"

#include <cmath>
#include <iterator>
#include <algorithm>

CompanionDisk::CompanionDisk(double R, uint64_t resolution, uint64_t window_size)
    : R_(R)
    , resolution_(resolution)
    , window_size_(window_size)
    , rays_length_(resolution_, 0.0)
    , min_ray_length_(0.0)
{
}

const std::vector<double>& CompanionDisk::get_pose() const
{
  return pose_;
}

const std::vector<double>& CompanionDisk::get_rays_length() const
{
  return rays_length_;
}

void CompanionDisk::update_pose(const std::vector<double>& robot_state,
                                const std::vector<double>& lidar_closest_point,
                                double distance)
{
    const double length = utils::norm2(lidar_closest_point, robot_state);
    const double ux = (robot_state[0] - lidar_closest_point[0]) / length;
    const double uy = (robot_state[1] - lidar_closest_point[1]) / length;

    pose_[0] = lidar_closest_point[0] + distance * ux;
    pose_[1] = lidar_closest_point[1] + distance * uy;
}

void CompanionDisk::update_rays_length(const std::vector<std::vector<double>>& lidar_points)
{
  for (size_t i = 0; i < resolution_; ++i) {
    rays_length_[i] = utils::norm2(pose_, lidar_points[i]);
  }

  auto min_it = std::min_element(rays_length_.begin(), rays_length_.end());
  min_arg_ = std::distance(rays_length_.begin(), min_it);
  min_ray_length_ = *min_it;
}

double CompanionDisk::get_min_ray_length() const
{
  return min_ray_length_;
}