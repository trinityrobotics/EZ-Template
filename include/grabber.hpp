#pragma once
#include "api.h"

#define GRABBER_STOP_TIMEOUT 75
#define GRABBER_SENSOR_TIMEOUT 5000
enum grabber_state {
  GRABBER_DOWN = -90,
  GRABBER_UP = 0
};
enum grabber_sensor {
  SENSOR_PRESSED = 0,
  SENSOR_RELEASED = 1
};
void reset_grabber();
void set_grabber_state(grabber_state input);
void wait_grabber();
int32_t wait_grabber_sensor();
void grabber_control();
