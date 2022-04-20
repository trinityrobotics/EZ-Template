/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"
#include "pros/misc.hpp"

using namespace ez;

void Drive::ez_auto_task() {
  while (true) {
    // Autonomous PID
    if (get_mode() == DRIVE)
      drive_pid_task();
    else if (get_mode() == TURN)
      turn_pid_task();
    else if (get_mode() == SWING)
      swing_pid_task();
    else if (get_mode() == GPS_TURN)
      gps_turn_pid_task();
    else if (get_mode() == GPS_DRIVE)
      gps_drive_pid_task();

    if (pros::competition::is_autonomous() && !util::AUTON_RAN)
      util::AUTON_RAN = true;
    else if (!pros::competition::is_autonomous())
      set_mode(DISABLE);

    pros::delay(util::DELAY_TIME);
  }
}

// Drive PID task
void Drive::drive_pid_task() {
  // Compute PID
  leftPID.compute(left_sensor());
  rightPID.compute(right_sensor());
  headingPID.compute(get_gyro());

  // pros::screen::erase();
  // pros::screen::print(pros::E_TEXT_SMALL, 6, "PID L/R: %f/%f\n", leftPID.output, rightPID.output);
  // pros::screen::print(pros::E_TEXT_SMALL, 7, "Sensor L/R: %f/%f\n", left_sensor(), right_sensor());

  // Compute slew
  double l_slew_out = slew_calculate(left_slew, left_sensor());
  double r_slew_out = slew_calculate(right_slew, right_sensor());

  // Clip leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  double l_drive_out = util::clip_num(leftPID.output, l_slew_out, -l_slew_out);
  double r_drive_out = util::clip_num(rightPID.output, r_slew_out, -r_slew_out);

  // Toggle heading
  double gyro_out = heading_on ? headingPID.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + gyro_out;
  double r_out = r_drive_out - gyro_out;

  // Set motors
  if (drive_toggle)
    set_tank(l_out, r_out);
}

// Turn PID task
void Drive::turn_pid_task() {
  // Compute PID
  turnPID.compute(get_gyro());

  // Clip gyroPID to max speed
  double gyro_out = util::clip_num(turnPID.output, max_speed, -max_speed);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (turnPID.constants.ki != 0 && (fabs(turnPID.get_target()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (get_turn_min() != 0)
      gyro_out = util::clip_num(gyro_out, get_turn_min(), -get_turn_min());
  }

  // Set motors
  if (drive_toggle)
    set_tank(gyro_out, -gyro_out);
}

// Swing PID task
void Drive::swing_pid_task() {
  // Compute PID
  swingPID.compute(get_gyro());

  // Clip swingPID to max speed
  double swing_out = util::clip_num(swingPID.output, max_speed, -max_speed);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (swingPID.constants.ki != 0 && (fabs(swingPID.get_target()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
    if (get_swing_min() != 0)
      swing_out = util::clip_num(swing_out, get_swing_min(), -get_swing_min());
  }

  if (drive_toggle) {
    // Check if left or right swing, then set motors accordingly
    if (current_swing == LEFT_SWING)
      set_tank(swing_out, 0);
    else if (current_swing == RIGHT_SWING)
      set_tank(0, -swing_out);
  }
}

// Drive PID task
void Drive::gps_drive_pid_task() {
  bool target_close = false;
  pros::c::gps_status_s_t gpsData = get_mgps_status();
  bool use_cached = get_mgps_error() < .02 ? false : true;
  double target_bearing = get_target_bearing();
  double target_distance = get_target_distance();
  // GPS has a good signal
  // if (target_distance < .05) {
  //   target_close = true;
  // }
  // Check to see if we need to drive forward or backward
  if (target_bearing > 90) {
    target_distance *= -1;
    target_bearing -= 180;
  } else if (target_bearing < -90 ) {
    // The target is behind us
    target_distance *= -1;
    target_bearing += 180;
  }
  GPSheadingPID.compute_delta(target_bearing);
  GPSdrivePID.compute_delta(target_distance);

  // Compute PID depending on whether GPS has signal

  
  // Compute slew
  // double l_slew_out = slew_calculate(left_slew, left_sensor());
  // double r_slew_out = slew_calculate(right_slew, right_sensor());

  // Clip leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  // double l_drive_out = util::clip_num(leftPID.output, l_slew_out, -l_slew_out);
  // double r_drive_out = util::clip_num(rightPID.output, r_slew_out, -r_slew_out);

  // Toggle heading
  // double gyro_out = heading_on ? headingPID.output : 0;
  double heading_out = target_close ? 0 : GPSheadingPID.output;

  // Combine heading and drive
  double l_out = GPSdrivePID.output + heading_out;
  double r_out = GPSdrivePID.output - heading_out;

  pros::screen::erase();
  pros::screen::print(pros::E_TEXT_SMALL, 1, "Driving...\n");
  pros::screen::print(pros::E_TEXT_SMALL, 2, "Error: %2.3f\n", get_mgps_error());
  pros::screen::print(pros::E_TEXT_SMALL, 3, "Chassis X: %2.2f, Y: %2.2f\n", gpsData.x, gpsData.y);
  pros::screen::print(pros::E_TEXT_SMALL, 4, "Target X: %2.2f, Y: %2.2f\n", target_x_, target_y_);
  pros::screen::print(pros::E_TEXT_SMALL, 5, "Heading Current: %3.2f; Target: %3.2f\n", get_heading(), get_target_heading());
  pros::screen::print(pros::E_TEXT_SMALL, 6, "Tartet distance: %3.2f, Bearing: %3.2f\n", target_distance, target_bearing);
  pros::screen::print(pros::E_TEXT_SMALL, 7, "Drive L/R: %f/%f, Heading: %f\n", l_out, r_out, heading_out);

  // Set motors
  if (drive_toggle)
    set_tank(l_out, r_out);
}

// GPS Turn PID task
void Drive::gps_turn_pid_task() {
  pros::c::gps_status_s_t gpsData = get_mgps_status();

  // Get cached value to start to figure out if we need to re-calcuate
  double target_bearing = get_target_bearing(true);
  // bool use_cached = fabs(target_bearing) > 10 ? true : false;
  double target_distance = get_target_distance();
  target_bearing = get_target_bearing();
  double target_heading = get_target_heading();

  // Compute PID
  GPSturnPID.compute_delta(target_bearing);

  // Clip gyroPID to max speed
  double heading_out = util::clip_num(GPSturnPID.output, max_speed, -max_speed);

  pros::screen::erase();
  pros::screen::print(pros::E_TEXT_SMALL, 1, "Turning...\n");
  pros::screen::print(pros::E_TEXT_SMALL, 2, "Error: %2.3f\n", get_mgps_error());
  pros::screen::print(pros::E_TEXT_SMALL, 3, "Chassis X: %1.3f, Y: %1.3f\n", gpsData.x, gpsData.y);
  pros::screen::print(pros::E_TEXT_SMALL, 4, "Target X: %2.2f, Y: %2.2f\n", target_x_, target_y_);
  pros::screen::print(pros::E_TEXT_SMALL, 5, "Heading Current: %3.2f; Target: %3.2f\n", get_heading(), target_heading);
  pros::screen::print(pros::E_TEXT_SMALL, 6, "Tartet distance: %3.2f, Bearing: %3.2f\n", target_distance, target_bearing);
  pros::screen::print(pros::E_TEXT_SMALL, 7, "PID Output: %f\n", heading_out);

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (GPSturnPID.constants.ki != 0 && (fabs(GPSturnPID.get_target()) > GPSturnPID.constants.start_i && fabs(GPSturnPID.error) < GPSturnPID.constants.start_i)) {
    if (get_turn_min() != 0)
      heading_out = util::clip_num(heading_out, get_turn_min(), -get_turn_min());
  }

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (fabs(GPSturnPID.error) < GPSturnPID.constants.start_i) {
    if (get_turn_min() != 0)
      heading_out = util::clip_num(heading_out, get_turn_min(), -get_turn_min());
  }

  // Set motors
  if (drive_toggle)
    set_tank(heading_out, -heading_out);
}