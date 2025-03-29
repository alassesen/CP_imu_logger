/** @file
 *  @brief LED Service sample
 */

/*
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ST_BLE_SENSOR_LED_SVC_H_
#define ST_BLE_SENSOR_LED_SVC_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

typedef enum { RED_LED, GREEN_LED, BLUE_LED } color_t;

void led_update(int number, bool turn_on);
int led_init(void);
void led_log_light(void);

#ifdef __cplusplus
}
#endif

#endif
