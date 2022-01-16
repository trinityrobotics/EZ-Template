/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"

using namespace ez;

/// . . .
// Both of these functions need to handle going backwards too!
/// . . .

// Finds shortest path to angle
double Drive::angle_to_point(double x_target, double y_target) {
  // Difference in target to current (legs of triangle)
  double x_error = x_target - x_pos;
  double y_error = y_target - y_pos;

  // Sign of legs
  int x_sgn = util::sgn(x_error);
  int y_sgn = util::sgn(y_error);

  int add_for_quadrant = 0;

  // Quadrant 1
  if (x_sgn == 1 && y_sgn == 1)
    add_for_quadrant = 0;
  // Quadrant 2
  else if (x_sgn == 1 && y_sgn == -1)
    add_for_quadrant = 90;
  // Quadrant 3
  else if (x_sgn == -1 && y_sgn == -1)
    add_for_quadrant = 180;
  // Quadrant 4
  else if (x_sgn == -1 && y_sgn == 1)
    add_for_quadrant = 270;

  // Angle bound to 0 to 360
  double angle_bound_360 = util::to_deg(atan(fabs(y_error) / fabs(x_error))) + add_for_quadrant;

  // add something to handle encoders later
  // needs to handle greater then 720
  double current = get_gyro();
  if (fabs(current) >= 360) {
    int sgn = util::sgn(current);
    current = (fabs(current) - 360) * sgn;
  }

  double output = 0;

  // If left turn
  if (fabs((360 + current) - angle_bound_360) > fabs(angle_bound_360 - current))
    output = angle_bound_360;
  // If right turn
  else
    output = angle_bound_360 - 360;

  // printf("Target: %f   Output: %f   Add: %i      Current%f\n", angle_bound_360, output, add_for_quadrant, current);
  return output;
}

// Distance to point
double Drive::distance_to_point(double x_target, double y_target) {
  // Difference in target to current (legs of traignel)
  double x_error = x_target - x_pos;
  double y_error = y_target - y_pos;

  double distance = util::hypot(x_error, y_error);

  return distance;
}