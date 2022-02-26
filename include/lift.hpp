#pragma once
#include "api.h"

const double liftkP = 1.0;
const double liftkI = 0.001;
const double liftkD = 0.1;

enum lift_state {
  LIFT_FREE = -1,
  LIFT_DOWN = 10,
  LIFT_PLACE = 300,
  LIFT_UP = LIFT_UP_DEGREES,
  LIFT_UP2 = 800,
  LIFT_PULL = 800,
};

void set_lift_exit();
void set_lift_speed(int input);
void set_lift(int input);
void reset_lift();
void set_lift_state(lift_state input);
void wait_lift();

void lift_control();