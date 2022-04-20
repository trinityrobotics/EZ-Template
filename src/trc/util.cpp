/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"

namespace trc {
namespace util {

double get_angle(double x1, double y1, double x2, double y2) {
  //
  // Use atan2 to get an angle between two points 
  //
  // double yDiff = (y2 - y1);
  // double y = cos(x2) * sin(yDiff);
  // double x = cos(x1) * sin(x2) - sin(x1) * cos(x2) * cos(yDiff);
  // This is a bearing off of 0 degrees (negative mean counter clockwise)
  // double degrees = atan2(y, x)/M_PI*180;

  double x = x2 - x1;
  double y = y2 - y1;
    // theta = math.atan2(dy, dx)
    // angle = math.degrees(theta)  // angle is in (-180, 180]
    // if angle < 0:
    //     angle = 360 + angle
    // return angle
  double theta_rad = atan2(y,x);
  double theta_deg = (theta_rad/M_PI*180); // + (theta_rad > 0 ? 0 : 360);
  if (theta_deg > 90)
  {
    theta_deg = 450 - theta_deg;
  } else {
    theta_deg = 90 - theta_deg;
  }
  return theta_deg;
}

double corrected_heading(double original_heading, double gps_offset) {
  double corrected_heading = original_heading - gps_offset;
  if (corrected_heading < 0) {
    return corrected_heading + 360;
  } else {
    return corrected_heading;
  }
}

double get_distance(double x1, double y1, double x2, double y2, double offset) {
  //
  // Return the hypotenuse
  //
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0) - offset;
}

double m_to_in(double meters) {
  //
  // Return the hypotenuse
  //
  return meters * 39.37;
}

double in_to_m(double inches) {
  //
  // Return the hypotenuse
  //
  return inches / 39.37;
}

}  // namespace util
}  // namespace ez
