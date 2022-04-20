/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <iostream>
#include <tuple>

#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "pros/motors.h"
#include "trc/util.hpp"

class Mgps {
 public:

  /**
   * GPS sensor.
   */
  std::vector<pros::Gps> mgps;

  /**
   * Creates a Drive Controller using internal encoders and GPS
   *
   * \param left_motor_ports
   *        Input {1, -2...}.  Make ports negative if reversed!
   * \param right_motor_ports
   *        Input {-3, 4...}.  Make ports negative if reversed!
   * \param imu_port
   *        Port the IMU is plugged into.
   * \param wheel_diameter
   *        Diameter of your drive wheels.  Remember 4" is 4.125"!
   * \param ticks
   *        Motor cartridge RPM
   * \param ratio
   *        External gear ratio, wheel gear / motor gear.
   * \param gps_port
   *        Port the GPS is plugged into.
   * \param gps_x_offset
   *        The x offset of the GPS sesnor.
   * \param gps_y_offset
   *        The y offset of the GPS sesnor.
   * \param gps_yaw_offset_
   *        The roll offset of the GPS sesnor.
   */
  Mgps(std::vector<int> gps_ports, std::vector<double> gps_x_offsets, std::vector<double> gps_y_offsets, std::vector<double> gps_yaw_offsets);

  /**
   * Wait until the GPS has caught side of the barcodes.
   */
  void wait_gps();

  /**
   * Gets the current heading of the chassis
   */
  double get_heading();

  /**
   * Gets the status from the most accurate gps
   */
  pros::c::gps_status_s_t get_mgps_status();

  /**
   * Gets the error from the most accurate gps
   */
  double get_mgps_error();

  /**
   * Gets the heading from the most accurate gps
   */
  double get_mgps_heading();

  /**
   * Gets the heading to the stored target location
   */
  double get_target_heading(bool use_cached = false);

  /**
   * Gets the heading to the provided target location
   */
  double get_target_heading(double target_x, double target_y);

  /**
   * Gets the heading to the target when in drive mode
   */
  double get_drive_bearing(bool use_cached = false);

  /**
   * Gets the relative heading to the target
   */
  double get_target_bearing(bool use_cached = false);

  /**
   * Gets the distance to the stored target location in meters
   */
  double get_target_distance();

  /**
   * Gets the distance to the given target in meters
   */
  double get_target_distance(double target_x, double target_y, double target_offset = 0);

  /**
   * Set the initital position of the chassis.
   */
  void set_position(double x, double y, double heading);

  /**
   * Set the target for the chassis.
   */
  void set_target(double input_x, double input_y, double offset = 0);

private:
  /**
   * GPS
   */
  std::vector<double> gps_yaw_offset_;
  double target_x_;
  double target_y_;
  double target_offset_;
  double current_heading_;
  double target_heading_;
};