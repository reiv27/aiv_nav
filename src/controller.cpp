#include "aiv_nav/controller.hpp"

Controller::Controller(std::unique_ptr<State> init_state, double linear_velocity, double angular_velocity, double R_min, double R, double ro_0)
: state_(std::move(init_state))
, linear_velocity_(linear_velocity)
, angular_velocity_(angular_velocity)
, R_min_(R_min)
, R_(R)
, ro_0_(ro_0)
{
}

void Controller::update()
{
  if (state_) {
    state_->handle(*this);
  }
}

void Controller::setState(std::unique_ptr<State> s)
{
  state_ = std::move(s);
}

const State& Controller::state() const
{
  return *state_;
}

void Controller::incrementCount()
{
  ++count_;
}

int Controller::count() const
{
  return count_;
}