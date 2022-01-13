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
  double offset;
  double ratio;
  double wheel_size;
private:
  std::function<double()> get_value_func;
  std::function<void()> reset_func;
};
