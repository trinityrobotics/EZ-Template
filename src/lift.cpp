#include "main.h"
#include "pros/adi.hpp"

const int SLOW_SPEED = 100;
const int FAST_SPEED = 150;

PID liftPID{4, 0, 0, 0, "Lift"};
pros::Motor lift_motor(LIFT_MOTOR_PORT, LIFT_MOTOR_GEARSET, LIFT_MOTOR_REVERSED, MOTOR_ENCODER_DEGREES);
int lift_max_speed = 150;
void set_lift_speed(int input) { lift_max_speed = abs(input); }
void set_lift(int input) { lift_motor = input; }
bool did_reset = false;
void set_lift_exit() { liftPID.set_exit_condition(80, 20, 300, 50, 500, 500); }

std::string lift_state_to_string(lift_state input) {
  switch (input) {
    case LIFT_DOWN:
      return "Down";
      break;
    case LIFT_PLACE:
      return "Mid";
      break;
    case LIFT_UP:
      return "Up";
      break;
    case LIFT_FREE:
      return "Up";
      break;
    default:
      return "Out of bounds lift state";
      break;
  }
}

lift_state current_lift_state = LIFT_DOWN;
void set_lift_state(lift_state input) {
  if (input != LIFT_FREE) {
    set_lift_speed(current_lift_state > input ? SLOW_SPEED : FAST_SPEED);
    liftPID.set_target(input);
  }
  current_lift_state = input;
  std::cout << "\nNew Lift State: " << lift_state_to_string(input);
}

void reset_lift() {
  pros::screen::print(pros::E_TEXT_SMALL, 1, "Resetting Lift...\n");
  long timer = 0;
  lift_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  lift_motor.move_velocity(-20);
  while (true) {
    bool check = lift_motor.get_actual_velocity() == 0 ? true : false;
    if (check) timer += util::DELAY_TIME;
    if (timer >= 250) {
      break;
    }
    pros::delay(util::DELAY_TIME);
  }
  lift_motor.move_velocity(0);
  lift_motor.tare_position();
}

void liftTask() {
  lift_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  double output = 0;
  long timer = 0;
  while (true) {
    if (current_lift_state != LIFT_FREE) {
      double current = lift_motor.get_position();
      double clipped_pid = util::clip_num(liftPID.compute(current), lift_max_speed, -lift_max_speed);

      if (current_lift_state == LIFT_DOWN) {
        if (current = 100)
          output = clipped_pid;
        else {
          bool check = (lift_motor.get_actual_velocity() == 0 && !pros::competition::is_disabled()) ? true : false;
          if (check) timer += util::DELAY_TIME;
          if (timer >= 250) {
            output = -3;
            if (!did_reset) reset_lift();
            did_reset = true;
            timer = 250;
          } else {
            output = -40;
          }
        }
      } else {
        timer = 0;
        did_reset = false;
        output = clipped_pid;
      }

      if (pros::competition::is_disabled()) timer = 0;

      set_lift(output);
    }

    pros::delay(util::DELAY_TIME);
  }
}
pros::Task lift_task(liftTask);

void wait_lift() {
  while (liftPID.exit_condition(lift_motor, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

void init_lift() {
  long timer = 0;
  bool lift_moving = true;
  lift_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  lift_motor.move_velocity(-lift_max_speed);
  while (lift_moving) {
    bool stopped = (lift_motor.get_actual_velocity() == 0 ? true : false);
    if (stopped) timer += util::DELAY_TIME;
    if (timer >= 10 * util::DELAY_TIME) {
      lift_motor.move_velocity(0);
      lift_moving = false;
      timer = 0;
    }
    pros::delay(util::DELAY_TIME);
  }
  did_reset = true;
  master.rumble("--");
}

bool last_l1 = 0;
bool last_r2 = 0;
void lift_control() {
  if (abs(master.get_analog(ANALOG_LEFT_Y)) > 10) {
    lift_motor.move(master.get_analog(ANALOG_LEFT_Y));
    set_lift_state(LIFT_FREE);
  } else if (current_lift_state == LIFT_FREE) {
    lift_motor.move_velocity(0);
  }
  if (master.get_digital_new_press(DIGITAL_UP)) {
      set_lift_state(LIFT_UP);
  }
  if (master.get_digital_new_press(DIGITAL_DOWN)) {
      set_lift_state(LIFT_DOWN);
  }
}