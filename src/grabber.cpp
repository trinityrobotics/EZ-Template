#include "main.h"
#include "pros/adi.hpp"

pros::Motor grabber_motor(GRABBER_MOTOR_PORT, GRABBER_MOTOR_GEARSET, GRABBER_MOTOR_REVERSED, MOTOR_ENCODER_DEGREES);
int grab_max_speed = 127;
void reset_grab() { grabber_motor.tare_position(); }

std::string grabber_state_to_string(grabber_state input) {
  switch (input) {
    case GRABBER_DOWN:
      return "Down";
      break;
    case GRABBER_UP:
      return "Up";
      break;
    default:
      return "Out of bounds grab state";
      break;
  }
}

bool grabber_moving = false;
void set_grabber_state(grabber_state new_grabber_state) {
  if (new_grabber_state == GRABBER_DOWN) {
    grabber_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
    grabber_motor.move_velocity(grab_max_speed);
  } else {
    // grabber_motor.move_absolute(0, grab_max_speed);
    grabber_motor.set_brake_mode(MOTOR_BRAKE_BRAKE);
    grabber_motor.move_velocity(-grab_max_speed);
  }
  grabber_moving = true;
  std::cout << "\nNew Grabber State: " << grabber_state_to_string(new_grabber_state);
}

void wait_grabber() {
  do {
    pros::delay(util::DELAY_TIME);
  } while (grabber_moving);
}

void grabberTask() {
  set_grabber_state(GRABBER_UP);
  reset_grab();
  long timer = 0;
  bool did_reset = false;
  while (true) {
    if (grabber_moving) {
      bool stopped = (grabber_motor.get_actual_velocity() == 0 ? true : false);
      if (stopped) timer += util::DELAY_TIME;
      if (timer >= GRABBER_STOP_TIMEOUT) {
        grabber_motor.move_velocity(0);
        grabber_moving = false;
        timer = 0;
      }
    }
    pros::delay(util::DELAY_TIME);
  }
}
pros::Task grabber_task(grabberTask);

void grabber_control() {
  if (master.get_digital_new_press(DIGITAL_R1)) {
    set_grabber_state(GRABBER_UP);
  }
  if (master.get_digital_new_press(DIGITAL_R2)) {
    set_grabber_state(GRABBER_DOWN);
  }
}