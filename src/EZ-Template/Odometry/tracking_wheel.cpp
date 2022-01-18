/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "tracking_wheel.hpp"

Tracking_Wheel::Tracking_Wheel(pros::Motor motor, double wheel_diameter, double ticks_per_rotation, double offset, double ratio) {
  get_value_func = std::bind(&pros::Motor::get_position, motor);
  reset_func = std::bind(&pros::Motor::tare_position, motor);
  set_constants(offset, ratio, wheel_diameter, 50.0 * (3600.0 / ticks_per_rotation));
}

Tracking_Wheel::Tracking_Wheel(pros::Rotation rotationSensor, double wheel_diameter, double ticks_per_rotation, double offset, double ratio) {
  get_value_func = std::bind(&pros::Rotation::get_position, rotationSensor);
  reset_func = std::bind(&pros::Rotation::reset_position, rotationSensor);
  set_constants(offset, ratio, wheel_diameter, ticks_per_rotation);
}

Tracking_Wheel::Tracking_Wheel(pros::ADIEncoder encoder, double wheel_diameter, double ticks_per_rotation, double offset, double ratio) {
  get_value_func = std::bind(&pros::ADIEncoder::get_value, encoder);
  reset_func = std::bind(&pros::ADIEncoder::reset, encoder);
  set_constants(offset, ratio, wheel_diameter, ticks_per_rotation);
}

double Tracking_Wheel::get_value() {
  if (pros::millis() < 1500) return 0;
  return get_value_func();
}

void Tracking_Wheel::reset_position() { reset_func(); }

void Tracking_Wheel::set_constants(double offset, double ratio, double wheel_size, double ticks) {
  this->offset = offset;
  this->ratio = ratio;
  this->wheel_size = wheel_size;
  this->ticks = ticks;
}
