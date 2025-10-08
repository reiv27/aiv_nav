#include <iostream>
#include "aiv_nav/controller.hpp"

std::string_view ModeA::name() const
{
  return "ModeA";
}

void ModeA::handle(Controller& ctrl)
{
  ctrl.increment_count();

  if (ctrl.count() >= 3) {
    ctrl.set_state(std::make_unique<ModeG>());
  }
}