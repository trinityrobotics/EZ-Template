/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"

using namespace ez;

void Drive::set_x(double x) {
  x_pos = x;
  target.x = x;
}

void Drive::set_y(double y) {
  y_pos = y;
  target.y = y;
}

void Drive::set_theta(double a) { set_angle(a); }

void Drive::reset_odom() { set_pose({0, 0, 0}); }

void Drive::set_pose(pose target) {
  set_theta(target.theta);
  set_x(target.x);
  set_y(target.y);
}

void Drive::tracking_task() {
  double l_current = 0, r_current = 0;
  double c_current = 0;
  double l = 0, r = 0, c = 0;  // delta distance
  double l_last = 0, r_last = 0, c_last = 0;
  double radius_r = 0, radius_c = 0, h = 0, h2 = 0;  // rad for big circle
  double beta = 0, alpha = 0, theta = 0;
  double Xx = 0, Yy = 0, Xy = 0, Yx = 0;
  bool is_three = false;
  while (true) {
    if (selected_constructor == ez::TRACKING_THREE_WHEEL_IMU || selected_constructor == ez::TRACKING_THREE_WHEEL_NO_IMU)
      is_three = true;

    l_current = left_sensor() / LEFT_TICK_PER_INCH;
    r_current = right_sensor() / RIGHT_TICK_PER_INCH;
    if (is_three) c_current = center_tracker->get_value() / CENTER_TICK_PER_INCH;

    l = l_current - l_last;
    r = r_current - r_last;
    c = c_current - c_last;

    l_last = l_current;
    r_last = r_current;
    c_last = c_current;

    double width = left_tracker->offset + right_tracker->offset;

    // diff between wheels for correcting turning
    theta = (l - r) / width;

    if (theta != 0) {
      radius_r = r / theta;
      beta = theta / 2.0;
      h = ((radius_r + right_tracker->offset) * sin(beta)) * 2.0;
      if (is_three) {
        radius_c = c / theta;
        h2 = (radius_c + center_tracker->offset) * 2 * sin(beta);
      }
    } else {
      h = l;
      h2 = 0;
      beta = 0;
    }

    alpha = angle_rad + beta;

    Xx = h2 * cos(alpha);
    Xy = h2 * -sin(alpha);
    Yx = h * sin(alpha);
    Yy = h * cos(alpha);

    x_pos += Xx + Yx;
    y_pos += Xy + Yy;
    angle_rad += theta;
    angle_deg = util::to_deg(angle_rad);

    if (!(selected_constructor != TRACKING_THREE_WHEEL_IMU || selected_constructor != TRACKING_THREE_WHEEL_NO_IMU || selected_constructor != TRACKING_TWO_WHEEL_IMU || selected_constructor != TRACKING_TWO_WHEEL_NO_IMU))
      tracking.suspend();

    pros::delay(1);
  }
}