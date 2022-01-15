/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"

class Tracking_Wheel {
 public:
  Tracking_Wheel(pros::Motor motor, double ratio, double wheel_size, double ticks, double offset = 0);
  Tracking_Wheel(pros::Rotation rotationSensor, double ratio, double wheel_size, double ticks, double offset = 0);
  Tracking_Wheel(pros::ADIEncoder encoder, double ratio, double wheel_size, double ticks, double offset = 0);
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
