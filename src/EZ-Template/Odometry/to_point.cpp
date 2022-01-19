/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <cmath>

#include "EZ-Template/drive/drive.hpp"

using namespace ez;

// Projected point from a target
pose Drive::vector_off_point(double added, double theta, double x_target, double y_target, e_direction direction) {
  /// this needs work
  double y_error = sin(util::to_rad(theta)) * added;
  double x_error = sqrt(pow(added, 2) - pow(y_error, 2));

  pose target;
  target.x = x_error + x_target;
  target.y = y_error + y_target;
  return target;
}

// Finds error in shortest angle to point
double Drive::angle_to_point(double x_target, double y_target, e_direction direction) {
  // Difference in target to current (legs of triangle)
  double x_error = x_target - x_pos;
  double y_error = y_target - y_pos;

  // Flips target when traveling backwards
  int add = direction == REV ? 180 : 0;

  // Flips target when traveling backwards
  if (util::sgn(x_error) != x_error_sgn && util::sgn(y_error) != y_error_sgn)
    add = add == 180 ? 0 : 180;

  // Displacement of error
  double error = util::wrap_angle((util::to_deg(atan2(x_error, y_error)) - add) - get_gyro());
  printf("Current (%.2f, %.2f)    Target (%.2f, %.2f)   Angle Error %.2f   Add %i\n", x_pos, y_pos, global_x_target, global_y_target, headingPID.error, add);

  return error;
}

// Find shortest distance to point
double Drive::distance_to_point(double x_target, double y_target, e_direction direction) {
  // Difference in target to current (legs of triangle)
  double x_error = (x_target - x_pos);
  double y_error = (y_target - y_pos);

  // Makes the distance negative if going backwards
  int sgn = direction == REV ? -1 : 1;
  if (util::sgn(x_error) != x_error_sgn && util::sgn(y_error) != y_error_sgn)
    sgn = sgn == 1 ? -1 : 1;

  // Hypotenuse of triangle (the distance we have to travel)
  double distance = util::hypot(x_error, y_error) * sgn;

  return distance;
}