#include "aiv_nav/companion_disk.hpp"

#include <cmath>

CompanionDisk::CompanionDisk(double R, uint64_t resolution)
    : R_(R)
    , resolution_(resolution)
    , rays_length_(resolution, 0.0)
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