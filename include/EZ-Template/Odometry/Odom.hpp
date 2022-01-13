#pragma once
#include "main.h"
#include "tracking_wheel.hpp"
class Odom
{
public:
  double x;
  double y;
  double angle;
  Odom(std::vector<Tracking_Wheel>);
};
