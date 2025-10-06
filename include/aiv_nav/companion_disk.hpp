#pragma once

#include <iostream>

class CompanionDisk
{
public:
  CompanionDisk(double R, int resolution);

  void update_disk_data(std::vector<double>& lidar_ranges, std::vector<double>& robot_pose);

  double get_disk_position() const;
  std::vector<double> get_rays_length() const;

private:
  double R_;
  contst double resolution_;

  std::vector<double> disk_pose_ = {0.0, 0.0};
  std::vector<double> rays_length_(resolution_, 0.0);
  double min_ray_length_ = 0.0;
};