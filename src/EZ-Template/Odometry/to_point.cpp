/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <cmath>

#include "EZ-Template/drive/drive.hpp"

using namespace ez;

//
pose Drive::vector_off_point(double added, double theta, double x_target, double y_target, bool is_backwards) {
  double y_error = sin(util::to_rad(theta)) * added;
  double x_error = sqrt(pow(added, 2) - pow(y_error, 2));

  pose target;
  target.x = x_error + x_target;
  target.y = y_error + y_target;
  return target;
}

// Finds shortest path to angle
double Drive::angle_to_point(double x_target, double y_target, bool is_backwards) {
  // Difference in target to current (legs of triangle)
  double x_error = x_target - x_pos;
  double y_error = y_target - y_pos;

  // double target = util::wrap_angle(util::to_deg(atan(x_error / y_error)));
  //int add = is_backwards ? 180 : 0;
  int add = 0;
  double target = util::wrap_angle(util::to_deg(atan2(x_error, y_error)) + add);
  // printf("Current (%.2f, %.2f)   Target (%.2f, %.2f)   %f\n", x_pos, y_pos, x_target, y_target, util::to_deg(atan2(x_error, y_error)));

  return target;
}

// Find shortest distance to point
double Drive::distance_to_point(double x_target, double y_target, bool is_backwards) {
  // Difference in target to current (legs of triangle)
  double x_error = (x_target - x_pos);
  double y_error = (y_target - y_pos);

  // Makes the distance negative if going backwards
  int sgn = is_backwards ? -1 : 1;
  if (util::sgn(x_error) != x_error_sgn && util::sgn(y_error) != y_error_sgn)
    sgn = sgn == 1 ? -1 : 1;
  // is_reversing = !is_reversing;

  // if (is_reversing)
  // sgn = sgn == 1 ? -1 : 1;

  // Hypotenuse of triangle (the distance we have to travel)
  double distance = util::hypot(x_error, y_error) * sgn;

  // x_error_sgn = util::sgn(x_error);
  // y_error_sgn = util::sgn(y_error);

  // printf("Distance: %f\n", distance);
  return distance;
}