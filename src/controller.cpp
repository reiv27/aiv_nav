#include "aiv_nav/controller.hpp"
#include "aiv_nav/companion_disk.hpp"

Controller::Controller(std::unique_ptr<State> init_state,
                       double linear_velocity,
                       double angular_velocity,
                       double rho_0,
                       double R_epsilon,
                       int resolution)
    : state_(std::move(init_state))
    , linear_velocity_(linear_velocity)
    , angular_velocity_(angular_velocity)
    , rho_0_(rho_0)
    , lidar_data_(resolution, 0.0)
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
}

const std::vector<double>& Controller::get_robot_state() const
{
  return robot_state_;
}

const std::vector<double>& Controller::get_lidar_data() const
{
  return lidar_data_;
}