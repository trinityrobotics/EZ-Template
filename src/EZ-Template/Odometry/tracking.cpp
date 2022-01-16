/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"

using namespace ez;

void Drive::set_x(double x) { x_pos = x; }
void Drive::set_y(double y) { y_pos = y; }

void Drive::set_theta(double a) {
  reset_gyro(a);
  angle = a;
}

void Drive::reset_odom() {
  set_theta(0);
  set_x(0);
  set_y(0);
}

void Drive::tracking_task() {
  double l_current = 0, r_current = 0;
  double l, r;
  double l_last = 0, r_last = 0;
  double radius_l = 0, h = 0;
  double beta = 0, alpha = 0, theta = 0;
  double x = 0, y = 0;
  while (true) {
    // l_current = LEFT_TICK_PER_INCH != 0 ? left_sensor() / LEFT_TICK_PER_INCH : 0;
    // r_current = RIGHT_TICK_PER_INCH != 0 ? right_sensor() / RIGHT_TICK_PER_INCH : 0;
    l_current = left_sensor() / LEFT_TICK_PER_INCH;
    r_current = right_sensor() / RIGHT_TICK_PER_INCH;

    l = l_current - l_last;
    r = r_current - r_last;

    double width = left_tracker->offset + right_tracker->offset;

    theta = (l - r) / width;

    if (theta != 0) {
      radius_l = l / theta;
      beta = theta / 2.0;
      h = ((radius_l + (width / 2.0)) * sin(beta)) * 2.0;
    } else {
      h = l;
      beta = 0;
    }

    alpha = angle + beta;

    x = h * sin(alpha);
    y = h * cos(alpha);

    x_pos += x;
    y_pos += y;
    angle += theta;

    printf("X: %f  Y: %f  A: %f\n", x_pos, y_pos, angle);

    if (!(selected_constructor != TRACKING_THREE_WHEEL_IMU || selected_constructor != TRACKING_THREE_WHEEL_NO_IMU || selected_constructor != TRACKING_TWO_WHEEL_IMU || selected_constructor != TRACKING_TWO_WHEEL_NO_IMU))
      tracking.suspend();

    pros::delay(1);
  }
}