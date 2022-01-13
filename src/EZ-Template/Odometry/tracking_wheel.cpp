#include "tracking_wheel.hpp"
Tracking_Wheel::Tracking_Wheel(pros::Motor motor)
{
  get_value_func = std::bind(&pros::Motor::get_position, motor);
  reset_func = std::bind(&pros::Motor::tare_position, motor);
}

Tracking_Wheel::Tracking_Wheel(pros::Rotation rotationSensor)
{
  get_value_func = std::bind(&pros::Rotation::get_position, rotationSensor);
  reset_func = std::bind(&pros::Rotation::reset_position, rotationSensor);
}
Tracking_Wheel::Tracking_Wheel(pros::ADIEncoder encoder)
{
  get_value_func = std::bind(&pros::ADIEncoder::get_value, encoder);
  reset_func = std::bind(&pros::ADIEncoder::reset, encoder);
}

double Tracking_Wheel::get_value()
{
  return get_value_func();
}
void Tracking_Wheel::reset_position()
{
  reset_func();
}
