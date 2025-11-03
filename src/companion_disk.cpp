#include "aiv_nav/companion_disk.hpp"

#include <cmath>
#include <iterator>
#include <algorithm>

#include "utils/utils.hpp"

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

void CompanionDisk::update_rays_length(const std::vector<std::vector<double>>& lidar_points,
                                       const std::vector<double>& lidar_data)
{
  const int n = resolution_;

  auto it_min = std::min_element(lidar_data.begin(), lidar_data.end());
  const int target = static_cast<int>(std::distance(lidar_data.begin(), it_min));

  for (size_t i = 0; i < resolution_; ++i) {
    rays_length_[i] = utils::norm2(pose_, lidar_points[i]);
  }
  
  auto mod = [n](int x){ int r = x % n; return r < 0 ? r + n : r; };
  const int start = mod(target - window_size_);
  const int end   = mod(target + window_size_);

  const double INF = std::numeric_limits<double>::infinity();

  if (start <= end) {
      for (int i = start; i < end; ++i) rays_length_[i] = INF;
  } else {
      for (int i = start; i < n; ++i) rays_length_[i] = INF;
      for (int i = 0; i < end;   ++i) rays_length_[i] = INF;
  }

  auto it = std::min_element(rays_length_.begin(), rays_length_.end());
  min_ray_length_ = *it;
  min_arg_ = static_cast<uint64_t>(std::distance(rays_length_.begin(), it));
}

double CompanionDisk::get_min_ray_length() const
{
  return min_ray_length_;
}

uint64_t CompanionDisk::get_min_arg() const
{
  return min_arg_;
}