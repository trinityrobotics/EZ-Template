#pragma once
#include "api.h"

#define INTAKE_STOP_TIMEOUT 100
#define INTAKE_REVERSE_TIME 3000
enum intake_state {
  INTAKE_REVERSE = -1,
  INTAKE_OFF = 0,
  INTAKE_FORWARD = 1,
  INTAKE_FAST = 2,
};

void set_intake_state(intake_state input);

void intake_control();