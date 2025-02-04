#include <stddef.h>
#include <stdio.h>
#include <zephyr/bluetooth/bluetooth.h>

extern int requested_state;

void bt_ready(int err);