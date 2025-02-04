/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

// #include <zephyr/fs/littlefs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include <fcntl.h>
#include <unistd.h>

#include "ble.h"
#include "imu.h"
#include "led.h"
#include "mass_storage_device.h"
#include "mpu9250.h"

extern int print_samples;
extern struct fs_mount_t fs_mnt;
extern int requested_state;
int current_state = 1;

LOG_MODULE_REGISTER(main);

#define MAX_PATH 128
#define SOME_FILE_NAME "some.dat"
#define SOME_DIR_NAME "some"
#define SOME_REQUIRED_LEN MAX(sizeof(SOME_FILE_NAME), sizeof(SOME_DIR_NAME))

void create_dummy_file(void);

int main(void) {
  struct fs_file_t file;
  char file_path[128];
  char text_to_file[255];

#ifdef CONFIG_MPU9250
  init_mpu9250_sensor();
#else
  init_imu_sensor();
#endif
  led_init();
  int ret;

  ret = bt_enable(bt_ready);
  if (ret) {
    LOG_ERR("Bluetooth init failed (err %d)", ret);
  }

  setup_disk();
  k_sleep(K_MSEC(500));

  LOG_INF("The device is put in USB mass storage mode.\n");

  double accel_x_out, accel_y_out, accel_z_out, gyro_x_out, gyro_y_out,
      gyro_z_out;
#ifdef CONFIG_MPU9250
  double mag_x_out, mag_y_out, mag_z_out;
#endif
  bool led_state_on = false;

  while (1) {
    led_state_on = !led_state_on;
    if (requested_state != current_state) {
      if (current_state == 2) {
        // Close File
        ret = fs_close(&file);
        if (ret < 0) {
          LOG_ERR("FAIL: close %s: %d", file_path, ret);
        } else {
          printk("fs_close - DONE\n");
        }
        fs_unmount(&fs_mnt);

        ret = usb_enable(NULL);
        if (ret != 0) {
          LOG_ERR("Failed to enable USB");
          return 0;
        }
      }

      if (requested_state == 2) {
        // Open file
        usb_disable();
        fs_mount(&fs_mnt);
        int base = strlen(DISK_MOUNT_PT);
        strncpy(file_path, DISK_MOUNT_PT, sizeof(file_path));
        file_path[base++] = '/';
        file_path[base] = 0;
        strcat(&file_path[base], "IMU_DATA.csv");
        fs_file_t_init(&file);
        if (fs_open(&file, file_path, FS_O_CREATE | FS_O_RDWR) != 0) {
          LOG_ERR("Failed to create file %s", file_path);
        } else {
          printk("fs_open - DONE\n");
#ifdef CONFIG_MPU9250
          snprintf(
              text_to_file, sizeof(text_to_file),
              "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z\n");
#else
          snprintf(text_to_file, sizeof(text_to_file),
                   "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n");
#endif
          fs_write(&file, text_to_file, strlen(text_to_file));
        }
      }

      current_state = requested_state;
    }

    if (current_state == 1) {
      // "Idle state"
      led_update(BLUE_LED, true);
      k_sleep(K_MSEC(250));
      led_update(BLUE_LED, false);
      k_sleep(K_MSEC(2500));
    } else if (current_state == 2) {
      // "Starting Logging"
#ifdef CONFIG_MPU9250
      get_mpu9250_values(&accel_x_out, &accel_y_out, &accel_z_out, &gyro_x_out,
                         &gyro_y_out, &gyro_z_out, &mag_x_out, &mag_y_out,
                         &mag_z_out);
      snprintf(text_to_file, sizeof(text_to_file),
               "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
               accel_x_out, accel_y_out, accel_z_out, gyro_x_out, gyro_y_out,
               gyro_z_out, mag_x_out, mag_y_out, mag_z_out);
#else
      get_imu_values(&accel_x_out, &accel_y_out, &accel_z_out, &gyro_x_out,
                     &gyro_y_out, &gyro_z_out);
      snprintf(text_to_file, sizeof(text_to_file),
               "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", accel_x_out, accel_y_out,
               accel_z_out, gyro_x_out, gyro_y_out, gyro_z_out);
#endif
      fs_write(&file, text_to_file, strlen(text_to_file));
      led_update(RED_LED, led_state_on);
      k_sleep(K_MSEC(100));
    } else if (current_state == 3) {
      // "Stop Logging"
      led_update(GREEN_LED, true);
      k_sleep(K_MSEC(250));
      led_update(GREEN_LED, false);
      k_sleep(K_MSEC(2500));
    }
  }
}