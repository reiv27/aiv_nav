#include "aiv_nav/controller.hpp"

#include <cmath>
#include <limits>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <iostream>
#include "aiv_nav/states.hpp"
#include "aiv_nav/companion_disk.hpp"
#include "aiv_nav/curvature.hpp"

Controller::Controller()
    : linear_velocity_(0.0)
    , angular_velocity_(0.0)
    , R_min_(0.0)
    , rho_0_(0.0)
    , R_vis_(0.0)
    , resolution_(0)
    , lidar_angle_offset_(0.0)
    , disk_()
    , nu_(0.0)
    , history_size_(0)
    , u_history_(0, 0.0)
    , curvature_points_(0,0)
{
}

Controller::Controller(double linear_velocity,
                       double angular_velocity,
                       double rho_0,
                       double R_epsilon,
                       double R_vis,
                       int resolution,
                       double lidar_angle_offset,
                       uint64_t window_size,
                       double nu,
                       int history_size,
                       int curvature_points_size)
    : linear_velocity_(linear_velocity)
    , angular_velocity_(angular_velocity)
    , R_min_{ linear_velocity / angular_velocity + R_epsilon }
    , rho_0_(rho_0)
    , R_vis_(R_vis)
    , resolution_(resolution)
    , lidar_angle_offset_(lidar_angle_offset)
    , disk_{ R_min_, resolution_, window_size }
    , nu_(nu)
    , history_size_(history_size)
    , u_history_(history_size, 0.0)
    , curvature_points_(2 * curvature_points_size + 1, 0.0)
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

  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  const double t_current = std::chrono::duration<double>(duration).count();
  dt_ = t_current - t_prev_;
  t_prev_ = t_current;

  double kappa = 0.0;
  curvature_valid_ = estimate_curvature(kappa);
  curvature_ = kappa;

  if (curvature_valid_) {
    std::cout << "kappa: " << kappa << std::endl;
  }

  u_ = state_->calculate_control_signal(*this);

  // double tau = 0.15;
  // double alpha = std::exp(-dt_ / tau);

  // u_ = alpha * u_prev_ + (1 - alpha) * u_;
  // u_prev_ = u_;
  // u_ = moving_average_(u_);
}

void Controller::set_state(std::unique_ptr<State> s)
{
  state_ = std::move(s);
}

const State& Controller::state() const
{
  return *state_;
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
  }

  disk_.update_pose(robot_state_, lidar_closest_point_, rho_0_ + R_min_);
  disk_.update_rays_length(lidar_points_, lidar_data);

}

const std::vector<double>& Controller::get_robot_state() const
{
  return robot_state_;
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

double Controller::get_dt()
{
  return dt_;
}

double Controller::get_dR_prev() const
{
  return dR_prev_;
}

void Controller::set_dR_prev(double dR_prev)
{
  dR_prev_ = dR_prev;
}

double Controller::get_nu() const
{
  return nu_;
}

double Controller::get_disk_min_ray_length() const
{
  return disk_.get_min_ray_length();
}

uint64_t Controller::get_disk_min_arg() const
{
  return disk_.get_min_arg();
}

void Controller::set_verA(const std::vector<double>& verA)
{
  verA_ = verA;
}

const std::vector<double>& Controller::get_verA() const
{
  return verA_;
}

void Controller::set_gap_points(const std::vector<double>& gap_point_1,
                                const std::vector<double>& gap_point_2)
{
  gap_point_1_ = gap_point_1;
  gap_point_2_ = gap_point_2;
}

const std::vector<double>& Controller::get_gap_point_1() const
{
  return gap_point_1_;
}

const std::vector<double>& Controller::get_gap_point_2() const
{
  return gap_point_2_;
}

double Controller::moving_average_(double u)
{
  u_history_.pop_front();
  u_history_.push_back(u);
  
  double sum = 0.0;
  for (const auto& val : u_history_) {
    sum += val;
  }
  return sum / history_size_;
}

bool Controller::estimate_curvature(double& kappa_out,
                                    double break_jump_m,
                                    int half_window) const
{
  kappa_out = 0.0;
  if (half_window < 0) {
    if (curvature_points_.size() < 3) {
      return false;
    }
    half_window = static_cast<int>((curvature_points_.size() - 1) / 2);
  }
  return aiv_nav::curvature::estimate_curvature(
    kappa_out,
    lidar_data_,
    lidar_points_,
    R_vis_,
    half_window,
    break_jump_m);
}

double Controller::get_curvature() const
{
  return curvature_;
}

bool Controller::is_curvature_valid() const
{
  return curvature_valid_;
}