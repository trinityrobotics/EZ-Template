#include "main.h"
#include "pros/adi.hpp"

pros::Motor intake_motor(INTAKE_MOTOR_PORT, INTAKE_MOTOR_GEARSET, INTAKE_MOTOR_REVERSED, MOTOR_ENCODER_DEGREES);
int intake_max_speed = 300;

std::string intake_state_to_string(intake_state input) {
  switch (input) {
    case INTAKE_REVERSE:
      return "Reverse";
      break;
    case INTAKE_OFF:
      return "Off";
      break;
    case INTAKE_FORWARD:
      return "Forward";
      break;
    case INTAKE_FAST:
      return "Forward";
      break;      
    default:
      return "Out of bounds intake state";
      break;
  }
}

intake_state current_intake_state = INTAKE_OFF;
long timer = 0;
bool unstopping_intake = false;
void set_intake_state(intake_state new_input_state) {
  // If the same button was pushed twice, then turn the intake off.
  if (new_input_state == current_intake_state) {
    current_intake_state = INTAKE_OFF;
    timer = 0;
    unstopping_intake = false;
  } else {
    current_intake_state = new_input_state;
    timer = 0;
    unstopping_intake = false;
  }
  std::cout << "\nNew Intake State: " << intake_state_to_string(new_input_state);
}

void intakeTask() {
  set_intake_state(INTAKE_OFF);
  intake_motor.set_brake_mode(MOTOR_BRAKE_BRAKE);
  while (true) {
    // The intake is stuck, so we'll reverse for a bit.
    if (unstopping_intake) {
      timer += util::DELAY_TIME;
      intake_motor.move_velocity(-intake_max_speed);
      if (timer >= INTAKE_REVERSE_TIME) {
        unstopping_intake = false;
        timer = 0;
      }
    } else {
      // Normal operation.  Use the enum value to determine direction.
      intake_motor.move_velocity(current_intake_state * intake_max_speed);
      // Check to see if the intake is stuck (stopped).
      bool stopped = (intake_motor.get_actual_velocity() == 0 ? true : false);
      if (current_intake_state == INTAKE_FORWARD && stopped) {
        timer += util::DELAY_TIME;
        if (timer >= INTAKE_STOP_TIMEOUT) {
          unstopping_intake = true;
          timer = 0;
        }
      }
    }
    pros::delay(util::DELAY_TIME);
  }
}
pros::Task intake_task(intakeTask);

void intake_control() {
  if (master.get_digital_new_press(DIGITAL_X)) {
    set_intake_state(INTAKE_FORWARD);
  }
  if (master.get_digital_new_press(DIGITAL_B)) {
    set_intake_state(INTAKE_REVERSE);
  }
}