#pragma once
#include "api.h"

#define HOARDER_STOP_TIMEOUT 75
enum hoarder_state {
  HOARDER_DOWN = 700,
  HOARDER_HOOK = 500,
  HOARDER_UP = 0
};

void reset_hoarder();
void wait_hoarder();
void set_hoarder_state(hoarder_state input);
void hoarder_control();