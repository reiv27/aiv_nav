#include "aiv_nav/controller.hpp"
#include "aiv_nav/companion_disk.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

Controller::Controller(std::unique_ptr<State> init_state,
                       double linear_velocity,
                       double angular_velocity,
                       double rho_0,
                       double R_epsilon,
                       double R_vis,
                       int resolution,
                       double lidar_angle_offset,
                       uint64_t window_size)
    : state_(std::move(init_state))
    , linear_velocity_(linear_velocity)
    , angular_velocity_(angular_velocity)
    , R_min_{ linear_velocity / angular_velocity + R_epsilon }
    , rho_0_(rho_0)
    , R_vis_(R_vis)
    , resolution_(resolution)
    , lidar_angle_offset_(lidar_angle_offset)
    , disk_{ R_min_, resolution_, window_size }
{
}

void Controller::update(const std::vector<double>& robot_state,
                        const std::vector<double>& lidar_data)
{
  set_robot_state_(robot_state);
  set_lidar_data_(lidar_data);

  if (state_) {
    state_->handle(*this);
  }

  // Debug v_
  if (state_->name() == "ModeC") {
    v_ = 0.0;
  }
  u_ = state_->calculate_control_signal(*this);
}

void Controller::set_state(std::unique_ptr<State> s)
{
  state_ = std::move(s);
}

const State& Controller::state() const
{
  return *state_;
}

void Controller::increment_count()
{
  ++count_;
}

int Controller::count() const
{
  return count_;  
}

double Controller::get_control_signal() const
{
  return u_;
}

void Controller::set_robot_state_(const std::vector<double>& robot_state)
{
  robot_state_ = robot_state;
}

void Controller::set_lidar_data_(const std::vector<double>& lidar_data)
{
  lidar_data_ = lidar_data;
  min_dist_ = std::numeric_limits<double>::infinity();
  lidar_points_.clear();

  size_t min_idx = 0;
  for (size_t i = 0; i < resolution_; ++i) {
    // Find min distance
    if (std::isfinite(lidar_data_[i]) && lidar_data_[i] < min_dist_) {
      min_dist_ = lidar_data_[i];
      min_idx = i;
    }

    // Calculate lidar points
    // indexes of lidar points are increasing in counter-clockwise direction
    const double angle = robot_state_[2] + lidar_angle_offset_ + i * 2 * M_PI / resolution_;
    const double range = std::min(lidar_data_[i], R_vis_);
    const double lidar_point_x = robot_state_[0] + range * std::cos(angle);
    const double lidar_point_y = robot_state_[1] + range * std::sin(angle);
    const std::vector<double> lidar_point = {lidar_point_x, lidar_point_y};
    lidar_points_.push_back(lidar_point);
    lidar_closest_point_ = lidar_points_[min_idx]; 

    disk_.update_pose(robot_state_, lidar_closest_point_, rho_0_ + R_min_);
  }
}

const std::vector<double>& Controller::get_robot_state() const
{
  return robot_state_;
}

const std::vector<double>& Controller::get_lidar_data() const
{
  return lidar_data_;
}

double Controller::get_min_dist() const
{
  return min_dist_;
}

const std::vector<std::vector<double>>& Controller::get_lidar_points() const
{
  return lidar_points_;
}

const std::vector<double>& Controller::get_closest_lidar_point() const
{
  return lidar_closest_point_;
}

double Controller::get_angular_velocity() const
{
  return angular_velocity_;
}

double Controller::get_linear_velocity() const
{
  return linear_velocity_;
}

double Controller::get_R_min() const
{
  return R_min_;
}

double Controller::get_rho_0() const
{
  return rho_0_;
}

const std::vector<double>& Controller::get_disk_pose() const
{
  return disk_.get_pose();
}