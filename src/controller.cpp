#include "aiv_nav/controller.hpp"

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