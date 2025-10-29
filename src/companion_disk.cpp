#include "aiv_nav/companion_disk.hpp"

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
    double dx = robot_state[0] - lidar_closest_point[0]; 
    double dy = robot_state[1] - lidar_closest_point[1];

    double length = std::sqrt(dx * dx + dy * dy);

    double ux = dx / length;
    double uy = dy / length;

    pose_[0] = lidar_closest_point[0] + distance * ux;
    pose_[1] = lidar_closest_point[1] + distance * uy;
}

void CompanionDisk::update_rays_length(const std::vector<std::vector<double>>& lidar_points)
{
  for (size_t i = 0; i < resolution_; ++i) {
    const double dx = lidar_points[i][0] - pose_[0];
    const double dy = lidar_points[i][1] - pose_[1];
    rays_length_[i] = std::sqrt(dx*dx + dy*dy);
  }

  auto min_it = std::min_element(rays_length_.begin(), rays_length_.end());
  min_arg_ = std::distance(rays_length_.begin(), min_it);
  min_ray_length_ = *min_it;
}