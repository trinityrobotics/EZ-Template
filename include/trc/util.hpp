/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <bits/stdc++.h>
#include <stdio.h>
#include <string.h>

#include "api.h"

namespace trc {
namespace util {
/**
 * Returns angle between two points
 */
double get_angle(double x1, double y1, double x2, double y2);

/**
 * Returns heading corrected for GPS offset
 */
double corrected_heading(double original_heading, double gps_offset);

/**
 * Returns angle between two points
 */
double get_distance(double x1, double y1, double x2, double y2, double offset = 0);

/**
 * Returns inches given meters
 */
double m_to_in(double meters);

/**
 * Returns meters given inches
 */
double in_to_m(double inches);

}  // namespace util
}  // namespace trc
