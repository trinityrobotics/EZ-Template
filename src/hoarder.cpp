#include "main.h"
#include "pros/adi.hpp"

using namespace ez;

pros::ADIDigitalOut horder_piston1(HOARDER_PISTON1);
pros::ADIDigitalOut horder_piston2(HOARDER_PISTON2);
//pros::Motor hoarder_motor(HOARDER_MOTOR_PORT, HOARDER_MOTOR_GEARSET, HOARDER_MOTOR_REVERSED, MOTOR_ENCODER_DEGREES);

std::string hoarder_state_to_string(hoarder_state input) {
  switch (input) {
    case HOARDER_DOWN:
      return "Down";
      break;
    case HOARDER_UP:
      return "Up";
      break;
    default:
      return "Out of bounds hoarder state";
      break;
  }
}

// hoarder_state target_hoarder_state = HOARDER_UP;
// bool hoarder_moving = false;
void set_hoarder_state(hoarder_state new_hoarder_state) {
//   if (new_hoarder_state == HOARDER_DOWN) {
//     hoarder_motor.set_brake_mode(MOTOR_BRAKE_COAST);
//     hoarder_motor.move_velocity(hoarder_max_speed);
//   } else {
//     // hoarder_motor.move_absolute(0, hoarder_max_speed);
//     hoarder_motor.set_brake_mode(MOTOR_BRAKE_BRAKE);
//     hoarder_motor.move_velocity(-hoarder_max_speed);
//   }
//   target_hoarder_state = new_hoarder_state;
//   hoarder_moving = true;
//   std::cout << "\nNew Hoarder State: " << hoarder_state_to_string(new_hoarder_state);
}

void wait_hoarder() {
//   do {
//     pros::delay(util::DELAY_TIME);
//   } while (hoarder_moving);
}

// void hoarderTask() {
//   set_hoarder_state(HOARDER_UP);
//   reset_hoarder();
//   long timer = 0;
//   bool did_reset = false;
//   while (true) {
//     if (hoarder_moving) {
//       bool stopped = (hoarder_motor.get_actual_velocity() == 0 ? true : false);
//       if (stopped) timer += util::DELAY_TIME;
//       if (target_hoarder_state == HOARDER_UP && hoarder_bumper.get_value()) {
//         hoarder_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
//         hoarder_motor.move_velocity(0);
//         hoarder_moving = false;
//         timer = 0;
//       } else if (timer >= HOARDER_STOP_TIMEOUT) {
//         hoarder_motor.move_velocity(0);
//         hoarder_moving = false;
//         timer = 0;
//       }
//     }
//     pros::delay(util::DELAY_TIME);
//   }
// }
// pros::Task hoarder_task(hoarderTask);

void hoarder_control() {
  if (master.get_digital_new_press(DIGITAL_L1)) {
    horder_piston1.set_value(true);
    horder_piston2.set_value(true);
    // set_hoarder_state(HOARDER_UP);
  }
  if (master.get_digital_new_press(DIGITAL_L2)) {
    horder_piston1.set_value(false);
    horder_piston2.set_value(false);
    // set_hoarder_state(HOARDER_DOWN);
  }
}