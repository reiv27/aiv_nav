#include "aiv_nav/companion_disk.hpp"

CompanionDisk::CompanionDisk(double R, int resolution)
    : R_(R)
    , resolution_(resolution)
    , rays_length_(resolution, 0.0)
{
}