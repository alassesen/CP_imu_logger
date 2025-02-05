#include <stddef.h>
#include <stdio.h>
#include <zephyr/bluetooth/bluetooth.h>

extern int requested_state;

typedef enum {
  SYSTEM_UNKNOWN_STATE,
  SYSTEM_IDLE,
  SYSTEM_LOGGING,
  SYSTEM_STOP_LOGGING,
  SYSTEM_OFF
} sys_state_t;

void bt_ready(int err);