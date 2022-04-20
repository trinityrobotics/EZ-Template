#include "main.h"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 127; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 80;
const int SWING_SPEED = 90;
const int SLOW_DRIVE_SPEED = 80;



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
  chassis.set_pid_constants(&chassis.GPSheadingPID, 3, 0, 5, 0);
  chassis.set_pid_constants(&chassis.GPSturnPID, 8, 0, 0, 0);
  chassis.set_pid_constants(&chassis.GPSdrivePID, 125, 0, 50, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.75, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.25, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 7, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

// void exit_condition_defaults() {
//   chassis.set_exit_condition(&chassis.turnPID, 100, 3, 500, 7, 500, 500);
//   chassis.set_exit_condition(&chassis.swingPID, 100, 3, 500, 7, 500, 500);
//   chassis.set_exit_condition(&chassis.forward_drivePID, 80, 5, 300, 8, 500, 500);
//   chassis.set_exit_condition(&chassis.backward_drivePID, 80, 5, 300, 8, 500, 500);
//   chassis.set_exit_condition(&chassis.GPSturnPID, 100, 2, 200, 4, 500, 500);
//   chassis.set_exit_condition(&chassis.GPSdrivePID, 200, .025, 300, .04, 500, 500);
// }

// void modified_exit_condition() {
//   chassis.set_exit_condition(&chassis.turnPID, 100, 3, 500, 7, 500, 500);
//   chassis.set_exit_condition(&chassis.GPSturnPID, 100, 3, 500, 7, 500, 500);
//   chassis.set_exit_condition(&chassis.swingPID, 100, 3, 500, 7, 500, 500);
//   chassis.set_exit_condition(&chassis.forward_drivePID, 80, 50, 300, 150, 500, 500);
//   chassis.set_exit_condition(&chassis.backward_drivePID, 80, 50, 300, 150, 500, 500);
// }
// Autnomous Skills
void skills_gps_auton1(void) {
  default_constants();
  exit_condition_defaults();
  set_grabber_state(GRABBER_UP);
  set_hoarder_state(HOARDER_UP);
  chassis.imu.set_heading(180);
  chassis.set_swing_pid(LEFT_SWING, 280, 75);
  chassis.wait_drive();
  // chassis.set_drive_pid(24, 75);
  // chassis.wait_drive();
  // chassis.wait_gps();  
  // pros::delay(500);
  // // Turn and drive to yellow base
  // chassis.set_gps_turn_pid(0, -.9, 50);
  // chassis.wait_drive();
  chassis.set_gps_drive_pid(0, -.9, 60, .20);
  wait_grabber_sensor();
  set_grabber_state(GRABBER_DOWN);
  pros::delay(500);
  // chassis.set_mode(DISABLE);
  set_lift_state(LIFT_UP);
  // Turn and drive to rings
  chassis.set_gps_turn_pid(-0.6, 0, 60);
  chassis.wait_drive();
  chassis.set_gps_drive_pid(-0.6, 0, 60);
  chassis.wait_drive();
  // // Turn and drive to red platform
  chassis.set_gps_drive_pid(-1.5, 0, 60, .30);
  chassis.wait_drive();
  // Turn and place yellow base on red platform
  // chassis.set_gps_turn_pid(-1.8, 0, 60);
  // chassis.wait_drive();
  set_lift_state(LIFT_PLACE); 
  set_grabber_state(GRABBER_UP);
  // Get right yellow base
  chassis.set_gps_turn_pid(0, .9, 60);
  chassis.wait_drive();
  set_lift_state(LIFT_DOWN);
  chassis.set_gps_drive_pid(0, .9, 60), .30;
  chassis.wait_drive();
  set_grabber_state(GRABBER_DOWN);
  set_lift_state(LIFT_UP);
  // Place yellow base on platform
  chassis.set_gps_turn_pid(-1, .6, 60);
  chassis.wait_drive();
  chassis.set_gps_drive_pid(-1, .6, 60);
  chassis.wait_drive(); 
  set_lift_state(LIFT_PLACE);
  wait_lift();
  set_grabber_state(GRABBER_UP);
  // Turn and get middle yellow base
  // chassis.set_gps_turn_pid(0, 0, 60);
  // chassis.wait_drive();

  // chassis.set_gps_drive_pid(0, 0, 60, .3);
  // chassis.wait_drive();  
  // set_grabber_state(GRABBER_DOWN);
  // set_lift_state(LIFT_UP);
  // Place middle yellow base on blue platform
  // chassis.set_gps_turn_pid(-1, -.5, 60);
  // chassis.wait_drive();
  // chassis.set_gps_drive_pid(-1, -.5, 60);
  // chassis.wait_drive();
  // set_lift_state(LIFT_PLACE); 
  // set_grabber_state(GRABBER_UP);
  // master.rumble("-- -- --");
}

// Autnomous Skills
void gps_test(void) {
  default_constants();
  exit_condition_defaults();
  // set_grabber_state(GRABBER_UP);
  // set_hoarder_state(HOARDER_UP);
  // chassis.set_position(1.5, -.95, 180);
  // chassis.imu.set_heading(180);
  // chassis.set_swing_pid(LEFT_SWING, 280, 75);
  // chassis.wait_drive();
  // chassis.set_drive_pid(24, 75);
  // chassis.wait_drive();
  // chassis.wait_gps();  
  // pros::delay(500);
  // // Turn and drive to yellow base
  chassis.set_gps_turn_pid(0, -.9, 60);
  chassis.wait_drive();
  chassis.set_gps_drive_pid(0, -.9, 60);
  chassis.wait_drive();
  // wait_grabber_sensor();
  // set_grabber_state(GRABBER_DOWN);
  // pros::delay(500);
  // chassis.set_mode(DISABLE);
  // set_lift_state(LIFT_UP);
  chassis.set_gps_turn_pid(-0.6, 0, 60);
  chassis.wait_drive();
  chassis.set_gps_drive_pid(-0.6, 0, 60);
  chassis.wait_drive();
  // chassis.set_gps_turn_pid(-1, 0, 60);
  // chassis.wait_drive();
  // chassis.set_gps_drive_pid(-1, 0, 60);
  // chassis.wait_drive();  
  master.rumble("-- -- --");
}

void gps_debug(void) {
  chassis.set_target(0, -.9);
  while (1) {
    double target_distance = chassis.get_target_distance();
    double target_bearing = chassis.get_target_bearing();
    // Check to see if we need to drive forward or backward
    if (target_bearing > 90) {
      target_distance *= -1;
      target_bearing -= 180;
    } else if (target_bearing < -90 ) {
      // The target is behind us
      target_distance *= -1;
      target_bearing += 180;
    }
    pros::c::gps_status_s_t gpsData = chassis.get_mgps_status();
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_SMALL, 1, "Debug...\n");
    pros::screen::print(pros::E_TEXT_SMALL, 2, "Error: %2.3f\n", chassis.get_mgps_error());
    pros::screen::print(pros::E_TEXT_SMALL, 3, "Chassis X: %2.3f, Y: %2.3f\n", gpsData.x, gpsData.y);
    // pros::screen::print(pros::E_TEXT_SMALL, 4, "Target X: %2.2f, Y: %2.2f\n", chassis.target_x_, chassis.target_y_);
    pros::screen::print(pros::E_TEXT_SMALL, 5, "Heading Current: %3.2f; Target: %3.2f\n", chassis.get_heading(), chassis.get_target_heading());
    pros::screen::print(pros::E_TEXT_SMALL, 6, "Tartet distance: %3.2f, Bearing: %3.2f\n", target_distance, chassis.get_target_bearing());
    pros::delay(20);
  }
}

void simple_test(void) {
  chassis.set_angle(180);
  // chassis.wait_drive();
  chassis.set_drive_pid(24, 25);
  chassis.wait_drive();
  chassis.set_drive_pid(-12, 25, false, false);
  chassis.wait_drive();
  // chassis.set_turn_pid(270, TURN_SPEED);
  master.rumble("-- -- --"); 
}

// Autnomous Skills
void skills_autonomous1(void) {
  chassis.set_angle(0);
  set_grabber_state(GRABBER_UP);
  set_lift_state(LIFT_DOWN);
  set_hoarder_state(HOARDER_UP);
  chassis.set_swing_pid(LEFT_SWING, 95, SWING_SPEED);
  chassis.wait_drive(); 
  set_intake_state(INTAKE_FORWARD);
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();  
  set_grabber_state(GRABBER_DOWN);
  set_lift_state(LIFT_UP);
  // chassis.set_turn_pid(117, TURN_SPEED);  
  // chassis.set_drive_pid(54, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_lift_state(LIFT_PLACE);
  // set_grabber_state(GRABBER_UP);
  // wait_grabber();
  // // yellow base on platform 40pts
  // chassis.set_drive_pid(-35, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_lift_state(LIFT_DOWN);
  // set_hoarder_state(HOARDER_DOWN);
  // wait_hoarder();
  // chassis.set_drive_pid(15, SLOW_DRIVE_SPEED);
  // chassis.wait_drive();
  // set_hoarder_state(HOARDER_UP);
  // chassis.set_turn_pid(-58, TURN_SPEED);
  // chassis.wait_drive();  
  // chassis.set_drive_pid(15, SLOW_DRIVE_SPEED);
  // chassis.wait_drive();
  // set_grabber_state(GRABBER_DOWN);
  // wait_grabber();
  // set_lift_state(LIFT_UP);
  // chassis.set_turn_pid(122, TURN_SPEED);
  // chassis.wait_drive();
  // chassis.set_drive_pid(56, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_grabber_state(GRABBER_UP);
  // wait_grabber();
  // chassis.set_drive_pid(-1, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // // red/blue base on platform from hoarder 80pts
  // chassis.set_turn_pid(3, TURN_SPEED);
  // chassis.wait_drive();
  // set_lift_state(LIFT_DOWN);
  // chassis.set_drive_pid(49, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_grabber_state(GRABBER_DOWN);
  // wait_grabber();
  // set_lift_state(LIFT_UP);
  // chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // chassis.set_turn_pid(-121.5, TURN_SPEED);
  // chassis.wait_drive();
  // set_intake_state(INTAKE_FAST);
  // chassis.set_drive_pid(92, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_lift_state(LIFT_PLACE);
  // set_grabber_state(GRABBER_UP);
  // wait_grabber();
  // // blue/red base on platform 120pts
  // chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // chassis.set_turn_pid(-218, TURN_SPEED);
  // chassis.wait_drive();
  // set_lift_state(LIFT_DOWN);
  // chassis.set_drive_pid(49, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_grabber_state(GRABBER_DOWN);
  // wait_grabber();
  // set_lift_state(LIFT_UP);
  // chassis.set_turn_pid(-48, TURN_SPEED);
  // chassis.wait_drive();
  // chassis.set_drive_pid(56, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_lift_state(LIFT_PLACE);
  // set_grabber_state(GRABBER_UP);
  // wait_grabber();
  // // yellow base on platform 160pts
  // chassis.set_drive_pid(-4, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // chassis.set_turn_pid(-179, TURN_SPEED);
  // chassis.wait_drive();
  // set_lift_state(LIFT_DOWN);
  // chassis.set_drive_pid(43, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_grabber_state(GRABBER_DOWN);
  // wait_grabber();
  // set_lift_state(LIFT_UP);
  // chassis.set_drive_pid(-4, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // chassis.set_turn_pid( -298 , TURN_SPEED);
  // chassis.wait_drive();
  // chassis.set_drive_pid(100, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // set_lift_state(LIFT_PLACE);
  // set_grabber_state(GRABBER_UP);
  // wait_grabber();
  // // big yellow base on platform 200pts
  // chassis.set_drive_pid(-6, DRIVE_SPEED);
  // chassis.wait_drive();
  // chassis.set_turn_pid( -265, TURN_SPEED);
  // chassis.wait_drive();
  // set_lift_state(LIFT_UP2);
  // chassis.set_drive_pid(-60, DRIVE_SPEED, true);
  // chassis.wait_drive();
  // // red/blue base on platform 220pts
  // master.rumble("-- -- --");
}

// Autnomous Skills
void skills_autonomous2(void) {
  chassis.set_angle(0);
  set_lift_state(LIFT_DOWN);
  set_hoarder_state(HOARDER_DOWN);
  wait_hoarder();
  pros::delay(500);
  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  pros::delay(500);
  set_hoarder_state(HOARDER_UP);
  wait_hoarder();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive(); 
  set_intake_state(INTAKE_FORWARD);
  chassis.set_drive_pid(46, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_grabber_state(GRABBER_DOWN);
  wait_grabber();
  pros::delay(200);
  set_lift_state(LIFT_UP);
  chassis.set_turn_pid(117, TURN_SPEED);  
  chassis.set_drive_pid(54, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_lift_state(LIFT_PLACE);
  pros::delay(200);
  set_grabber_state(GRABBER_UP);
  wait_grabber();
  // yellow base on platform 40pts
  chassis.set_drive_pid(-35, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_lift_state(LIFT_DOWN);
  set_hoarder_state(HOARDER_DOWN);
  wait_hoarder();
  chassis.set_drive_pid(15, SLOW_DRIVE_SPEED);
  chassis.wait_drive();
  set_hoarder_state(HOARDER_UP);
  chassis.set_turn_pid(-58, TURN_SPEED);
  chassis.wait_drive();  
  chassis.set_drive_pid(15, SLOW_DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(200);
  set_grabber_state(GRABBER_DOWN);
  wait_grabber();
  pros::delay(200);
  set_lift_state(LIFT_UP);
  chassis.set_turn_pid(122, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(56, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_grabber_state(GRABBER_UP);
  wait_grabber();
  chassis.set_drive_pid(-1, DRIVE_SPEED, true);
  chassis.wait_drive();
  // red/blue base on platform from hoarder 80pts
  chassis.set_turn_pid(3, TURN_SPEED);
  chassis.wait_drive();
  set_lift_state(LIFT_DOWN);
  pros::delay(400);
  chassis.set_drive_pid(49, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_grabber_state(GRABBER_DOWN);
  wait_grabber();
  pros::delay(200);
  set_lift_state(LIFT_UP);
  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-121.5, TURN_SPEED);
  chassis.wait_drive();
  set_intake_state(INTAKE_FAST);
  chassis.set_drive_pid(92, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_lift_state(LIFT_PLACE);
  pros::delay(200);
  set_grabber_state(GRABBER_UP);
  wait_grabber();
  // blue/red base on platform 120pts
  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-218, TURN_SPEED);
  chassis.wait_drive();
  set_lift_state(LIFT_DOWN);
  chassis.set_drive_pid(49, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_grabber_state(GRABBER_DOWN);
  wait_grabber();
  set_lift_state(LIFT_UP);
  chassis.set_turn_pid(-48, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(56, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_lift_state(LIFT_PLACE);
  pros::delay(200);
  set_grabber_state(GRABBER_UP);
  wait_grabber();
  // yellow base on platform 160pts
  chassis.set_drive_pid(-4, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-179, TURN_SPEED);
  chassis.wait_drive();
  set_lift_state(LIFT_DOWN);
  pros::delay(200);
  chassis.set_drive_pid(43, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_grabber_state(GRABBER_DOWN);
  wait_grabber();
  set_lift_state(LIFT_UP);
  chassis.set_drive_pid(-4, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid( -298 , TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(100, DRIVE_SPEED, true);
  chassis.wait_drive();
  set_lift_state(LIFT_PLACE);
  set_grabber_state(GRABBER_UP);
  wait_grabber();
  // big yellow base on platform 200pts
  chassis.set_drive_pid(-6, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid( -265, TURN_SPEED);
  chassis.wait_drive();
  set_lift_state(LIFT_UP2);
  chassis.set_drive_pid(-60, DRIVE_SPEED, true);
  chassis.wait_drive();
  // red/blue base on platform 220pts
  master.rumble("-- -- --");
  }

void comp_autonomous1() {
  chassis.imu.set_heading(0);
  set_grabber_state(GRABBER_UP);
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(50, DRIVE_SPEED, true, false);
  chassis.wait_drive();
  set_lift_state(LIFT_DOWN);
  set_grabber_state(GRABBER_DOWN);
  set_lift_state(LIFT_UP);
  chassis.set_drive_pid(-32, DRIVE_SPEED, true, false);
  chassis.wait_drive();
  set_hoarder_state(HOARDER_DOWN);
  chassis.set_turn_pid( -90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-11, DRIVE_SPEED, true, false);
  chassis.wait_drive();
  set_hoarder_state(HOARDER_UP);
  set_intake_state(INTAKE_FORWARD);
  chassis.set_turn_pid( 0, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid( 40, SLOW_DRIVE_SPEED, true, false);
  chassis.wait_drive();
  chassis.set_drive_pid( -45, SLOW_DRIVE_SPEED, true, false);
  chassis.wait_drive();
}
void comp_autonomous2() {
  chassis.imu.set_heading(0);
  set_hoarder_state(HOARDER_UP);
  set_grabber_state(GRABBER_UP);
  set_lift_state(LIFT_DOWN);
  set_intake_state(INTAKE_FORWARD);
  chassis.set_swing_pid( RIGHT_SWING, 90, SWING_SPEED);
  chassis.wait_drive();
  master.rumble("-- -- --");
  chassis.set_drive_pid(48, DRIVE_SPEED, true);
  chassis.wait_drive();  
  set_grabber_state(GRABBER_DOWN);
  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid( 180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(52, SLOW_DRIVE_SPEED, true);
  chassis.wait_drive();
}
///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches

  chassis.set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED);
  chassis.wait_drive();

  // chassis.set_drive_pid(-12, DRIVE_SPEED);
  // chassis.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at
  one_mogo_constants();
  set_grabber_state(GRABBER_DOWN);
  wait_grabber();
  pros::delay(500);
  set_lift_state(LIFT_UP);
  wait_lift();
  pros::delay(1000);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}

// . . .
// Make your own autonomous functions here!
// . . .