#include <iostream>
#include "aiv_nav/controller.hpp"

std::string_view ModeG::name() const
{
  return "ModeG";
}

void ModeG::handle(Controller& ctrl)
{

}

double ModeG::calculate_control_signal(const Controller& ctrl)
{
  return 0.0;
}