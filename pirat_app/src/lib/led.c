/** @file
 *  @brief Button Service sample
 */

/*
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "led.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led);

static const struct gpio_dt_spec red_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec green_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec blue_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static bool led_ok;

void led_update(int number, bool turn_on) {
  if (!led_ok) {
    return;
  }

  gpio_pin_set(red_led.port, red_led.pin, 0);
  gpio_pin_set(green_led.port, green_led.pin, 0);
  gpio_pin_set(blue_led.port, blue_led.pin, 0);

  if (turn_on) {
    if (number == RED_LED) {
      gpio_pin_set(red_led.port, red_led.pin, 1);
    } else if (number == GREEN_LED) {
      gpio_pin_set(green_led.port, green_led.pin, 1);
    } else if (number == BLUE_LED) {
      gpio_pin_set(blue_led.port, blue_led.pin, 1);
    }
  }
}

int led_init(void) {
  int ret;

  led_ok = gpio_is_ready_dt(&red_led);
  if (!led_ok) {
    LOG_ERR("Error: LED on GPIO %s pin %d is not ready", red_led.port->name,
            red_led.pin);
    return -ENODEV;
  }
  led_ok = gpio_is_ready_dt(&green_led);
  if (!led_ok) {
    LOG_ERR("Error: LED on GPIO %s pin %d is not ready", green_led.port->name,
            green_led.pin);
    return -ENODEV;
  }
  led_ok = gpio_is_ready_dt(&blue_led);
  if (!led_ok) {
    LOG_ERR("Error: LED on GPIO %s pin %d is not ready", blue_led.port->name,
            blue_led.pin);
    return -ENODEV;
  }

  ret = gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    LOG_ERR("Error %d: failed to configure GPIO %s pin %d", ret,
            red_led.port->name, red_led.pin);
  }
  ret = gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    LOG_ERR("Error %d: failed to configure GPIO %s pin %d", ret,
            green_led.port->name, green_led.pin);
  }
  ret = gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    LOG_ERR("Error %d: failed to configure GPIO %s pin %d", ret,
            blue_led.port->name, blue_led.pin);
  }

  return ret;
}

void led_log_light(void) {
  static uint8_t led_idx = 0;
  if (!led_ok) {
    return;
  }

  gpio_pin_set(red_led.port, red_led.pin, 0);
  led_idx = (led_idx + 1) % 5;
  if (led_idx == 0)
    gpio_pin_set(red_led.port, red_led.pin, 1);
}