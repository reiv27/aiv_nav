#pragma once

#include <iostream>

class CompanionDisk
{
public:
  CompanionDisk(double R_min, double R, double ro_0);
private:
  double R;
  double resolution_;
};