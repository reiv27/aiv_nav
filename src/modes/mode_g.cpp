#include <iostream>
#include "aiv_nav/controller.hpp"

std::string_view ModeG::name() const
{
  return "ModeG";
}

void ModeG::handle(Controller& ctx)
{
  ctx.incrementCount();

  if (ctx.count() >= 6) {
    ctx.setState(std::make_unique<ModeC>());
  }
}