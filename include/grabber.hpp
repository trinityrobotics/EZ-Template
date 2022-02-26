#pragma once
#include "api.h"

#define GRABBER_STOP_TIMEOUT 75
enum grabber_state {
  GRABBER_DOWN = -90,
  GRABBER_UP = 0
};

void reset_grabber();
void set_grabber_state(grabber_state input);
void wait_grabber();
void grabber_control();
