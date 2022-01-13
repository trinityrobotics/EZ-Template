#include "tracking_wheel.hpp"
Tracking_Wheel::Tracking_Wheel(pros::Motor motor)
{
//  getValueFunc = std::bind(&pros::Motor::get_position, motor);
  auto resetFunc = std::bind(&pros::Motor::tare_position, motor);
}

Tracking_Wheel::Tracking_Wheel(pros::Rotation rotationSensor)
{
  //getValueFunc = std::bind(&pros::Rotation::get_position, rotationSensor);
//  resetFunc = std::bind(&pros::Rotation::reset_position, rotationSensor));
}
Tracking_Wheel::Tracking_Wheel(pros::ADIEncoder encoder)
{
  //getValueFunc = std::bind(&pros::ADIEncoder::get_value, encoder);
//  resetFunc = std::bind(&pros::ADIEncoder::reset, encoder));
}
