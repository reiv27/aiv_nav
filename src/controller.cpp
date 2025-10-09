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
                       int resolution)
    : state_(std::move(init_state))
    , linear_velocity_(linear_velocity)
    , angular_velocity_(angular_velocity)
    , rho_0_(rho_0)
    , R_vis_(R_vis)
    , resolution_(resolution)
    , disk_(R_min_, resolution)
{
  R_min_ = linear_velocity / angular_velocity + R_epsilon;
}

void Controller::update(const std::vector<double>& robot_state, const std::vector<double>& lidar_data)
{
  set_robot_state_(robot_state);
  set_lidar_data_(lidar_data);



  if (state_) {
    state_->handle(*this);
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
  for (size_t i = 0; i < resolution_; ++i) {
    // Find min distance
    if (std::isfinite(lidar_data_[i]) && lidar_data_[i] < min_dist_) {
      min_dist_ = lidar_data_[i];
    }

    // Calculate lidar points
    if (lidar_data_[i] < R_vis_) {
      double lidar_point_x = lidar_data_[i] * std::cos(robot_state_[2] + i * 2 * M_PI / resolution_);
      double lidar_point_y = lidar_data_[i] * std::sin(robot_state_[2] + i * 2 * M_PI / resolution_);
      std::vector<double> lidar_point = {lidar_point_x, lidar_point_y};
      lidar_points_.push_back(lidar_point);
    }
    else {
      // Calculate lidar points if lidar data > R_vis_
      double lidar_point_x = R_vis_ * std::cos(robot_state_[2] + i * 2 * M_PI / resolution_);
      double lidar_point_y = R_vis_ * std::sin(robot_state_[2] + i * 2 * M_PI / resolution_);
      std::vector<double> lidar_point = {lidar_point_x, lidar_point_y};
      lidar_points_.push_back(lidar_point);
    }
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

const double& Controller::get_min_dist() const
{
  return min_dist_;
}

const std::vector<std::vector<double>>& Controller::get_lidar_points() const
{
  return lidar_points_;
}