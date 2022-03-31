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

// Drive PID task
void Drive::gps_drive_pid_task() {
  // Get GPS Data
  pros::c::gps_status_s_t gpsData = gps.get_status();
  double current_heading = fmod(gps.get_heading() - gps_yaw_offset, 360);
  double target_heading = ez::util::get_angle(gpsData.x, gpsData.y, target_x, target_y);
  double target_distance = ez::util::get_distance(gpsData.x, gpsData.y, target_x, target_y) * 100;

  // if (abs(target_heading - current_heading) < 10 ) {
  //   headingPID.set_target(target_heading);
  // }
  
  // Compute PID
  distancePID.compute_distance(target_distance * 100);
  headingPID.set_target(target_heading);
  headingPID.compute(current_heading);

  double drive_speed = util::clip_num(distancePID.output, max_speed, -max_speed);
  double turn_speed = util::clip_num(headingPID.output, 10, -10);

  // Combine heading and drive
  double l_out = drive_speed + turn_speed;
  double r_out = drive_speed - turn_speed;
  // double l_out = speed;
  // double r_out = speed;

  pros::screen::print(pros::E_TEXT_SMALL, 1, "Driving...\n");
  pros::screen::print(pros::E_TEXT_SMALL, 2, "Error: %4.2f\n", gps.get_error());
  pros::screen::print(pros::E_TEXT_SMALL, 3, "Position X: %1.2f, Y: %1.2f\n", gpsData.x, gpsData.y);
  pros::screen::print(pros::E_TEXT_SMALL, 4, "Heading Current: %3.2f; Target: %3.2\n", current_heading, target_heading);
  pros::screen::print(pros::E_TEXT_SMALL, 5, "Distance Target: %1.2f\n", target_distance);
  pros::screen::print(pros::E_TEXT_SMALL, 6, "Drive: %3.0f, Heading: %3.2f\n", drive_speed, turn_speed);

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


// GPS Turn PID task
void Drive::gps_turn_pid_task() {
  // Get GPS Data
  pros::c::gps_status_s_t gpsData = gps.get_status();
  double current_heading = fmod(gps.get_heading() - gps_yaw_offset, 360);
  double target_heading = ez::util::get_angle(gpsData.x, gpsData.y, target_x, target_y);
  double target_distance = ez::util::get_distance(gpsData.x, gpsData.y, target_x, target_y);



  if (abs(target_heading - current_heading) < 10 ) {
    turnPID.set_target(target_heading);
  }

  // Compute PID
  turnPID.compute(current_heading);

  // Clip gyroPID to max speed
  double gyro_out = util::clip_num(turnPID.output, max_speed, -max_speed);

  pros::screen::erase();
  pros::screen::print(pros::E_TEXT_SMALL, 2, "Error: %f\n", gps.get_error());
  pros::screen::print(pros::E_TEXT_SMALL, 3, "Position X: %f, Y: %f\n", gpsData.x, gpsData.y);
  pros::screen::print(pros::E_TEXT_SMALL, 4, "Heading Current: %f; Target: %f\n", current_heading, target_heading);
  pros::screen::print(pros::E_TEXT_SMALL, 5, "Distance Target: %f\n", target_distance);
  pros::screen::print(pros::E_TEXT_SMALL, 6, "PID Output: %f\n", gyro_out);

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
