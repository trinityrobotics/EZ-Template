/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"

using namespace ez;

// Finds shortest path to angle
double Drive::angle_to_point(double x_target, double y_target, bool is_backwards) {
  // Difference in target to current (legs of triangle)
  double x_error = x_target - x_pos;
  double y_error = y_target - y_pos;

  // Sign of legs (makes them TRUE if positive, and FALSE if negative)
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

  // When facing backwards, flip target angle by 180
  if (is_backwards) {
    if (angle_bound_360 > 180)
      angle_bound_360 -= 180;
    else
      angle_bound_360 += 180;
  }

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

// Find shortest distance to point
double Drive::distance_to_point(double x_target, double y_target, bool is_backwards) {
  // Difference in target to current (legs of triangle)
  double x_error = fabs(x_target - x_pos);
  double y_error = fabs(y_target - y_pos);

  // Makes the distance negative if going backwards
  int sgn = is_backwards ? -1 : 1;

  // Hypotenuse of triangle (the distance we have to travel)
  double distance = util::hypot(x_error, y_error) * sgn;

  // printf("Distance: %f\n", distance);
  return distance;
}