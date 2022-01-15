/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"

using namespace ez;

void Drive::set_x(double x) { x_pos = x; }
void Drive::set_y(double y) { y_pos = y; }

void Drive::set_theta(double a) {
  reset_gyro(a);
  theta = a;
}

void Drive::reset_odom() {
  theta = 0;
  x_pos = 0;
  y_pos = 0;
}

void Drive::tracking() {
  // reutnr x y 
}