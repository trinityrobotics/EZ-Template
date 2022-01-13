#pragma once
#include "main.h"
class Tracking_Wheel
{
public:
  Tracking_Wheel(pros::Motor motor);
  Tracking_Wheel(pros::Rotation rotationSensor);
  Tracking_Wheel(pros::ADIEncoder encoder);
  double get_value();
  void reset_position();
private:
//  std::_Bind<double()> getValueFunc;
//  std::_Bind<void()> resetFunc;
};
