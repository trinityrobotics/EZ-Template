/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"

class Tracking_Wheel {
 public:
  Tracking_Wheel(pros::Motor motor, double wheel_diameter, double ticks_per_rotation, double offset = 0, double ratio = 1);
  Tracking_Wheel(pros::Rotation rotationSensor, double wheel_diameter, double ticks_per_rotation, double offset = 0, double ratio = 1);
  Tracking_Wheel(pros::ADIEncoder encoder, double wheel_diameter, double ticks_per_rotation, double offset = 0, double ratio = 1);
  double get_value();
  void reset_position();
  double offset;
  double ratio;
  double wheel_size;
  double ticks;

 private:
  void set_constants(double offset, double ratio, double wheel_size, double ticks);
  std::function<double()> get_value_func;
  std::function<void()> reset_func;
};
