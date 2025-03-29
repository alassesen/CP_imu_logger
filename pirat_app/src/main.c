/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "ble.h"
#include "imu.h"
#include "led.h"
#include "mass_storage_device.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* State Management */
#define SYSTEM_IDLE 1
#define SYSTEM_LOGGING 2
#define SYSTEM_STOP_LOGGING 3
#define SYSTEM_OFF 4

/* Filename Management */
#define MAX_FILENAME_SIZE 128
char filename[MAX_FILENAME_SIZE];

/* Buffer for File Writing */
#define WRITE_BUFFER_SIZE 4096
static char write_buffer[WRITE_BUFFER_SIZE];
static size_t buffer_pos = 0;

/* System State Variables */
int print_samples;
extern struct fs_mount_t fs_mnt;
int requested_state;
int current_state = SYSTEM_IDLE; // Start in IDLE state

/* Function Declarations */
void flush_buffer(struct fs_file_t *file);
void disable_usb_mass_storage(void);

/* --- Function Implementations --- */

void flush_buffer(struct fs_file_t *file) {
  if (buffer_pos > 0) {
    int ret = fs_write(file, write_buffer, buffer_pos);
    if (ret < 0) {
      LOG_ERR("Write failed: %d", ret);
    }
    ret = fs_sync(file);
    if (ret < 0) {
      LOG_ERR("Sync failed: %d", ret);
    }
    buffer_pos = 0;
  }
}

void disable_usb_mass_storage(void) { usb_disable(); }

static int start_logging(struct fs_file_t *file, char *file_path) {
  struct fs_dir_t dir;

  fs_dir_t_init(&dir);
  fs_opendir(&dir, fs_mnt.mnt_point);

  fs_mount(&fs_mnt);
  int base = strlen(DISK_MOUNT_PT);
  strncpy(file_path, DISK_MOUNT_PT, 128); // Ensure size limit is respected
  file_path[base++] = '/';
  file_path[base] = 0;
  strcat(&file_path[base], filename);

  fs_file_t_init(file);
  if (fs_open(file, file_path, FS_O_CREATE | FS_O_RDWR | FS_O_APPEND) != 0) {
    LOG_ERR("Failed to create file %s", file_path);
    return -EIO;
  } else {
    LOG_INF("Opened file %s for logging", file_path);

    /* Write header to file */
    char header[] = "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,magn_x,magn_y,magn_"
                    "z\n"; // Include magnetometer
    size_t len = strlen(header);
    if (buffer_pos + len >= WRITE_BUFFER_SIZE) {
      flush_buffer(file);
    }
    memcpy(write_buffer + buffer_pos, header, len);
    buffer_pos += len;
    flush_buffer(file);
    return 0;
  }
}

static int stop_logging(struct fs_file_t *file, char *file_path) {
  flush_buffer(file);
  int ret = fs_close(file);
  if (ret < 0) {
    LOG_ERR("FAIL: close %s: %d", file_path, ret);
    return ret;
  } else {
    LOG_INF("Closed file %s", file_path);
  }

  fs_unmount(&fs_mnt);

  ret = usb_enable(NULL);
  if (ret != 0) {
    LOG_ERR("Failed to enable USB");
    return ret;
  }
  return 0;
}

int main(void) {
  struct fs_file_t file;
  char file_path[128];

  double accel_x_out, accel_y_out, accel_z_out, gyro_x_out, gyro_y_out,
      gyro_z_out;
  double magn_x_out, magn_y_out, magn_z_out; // Magnetometer
  int ret;

  /* Initialize Subsystems */
  LOG_INF("Starting application");

  /* Initialize filename (default) */
  strncpy(filename, "IMU_DATA.csv",
          MAX_FILENAME_SIZE); // Ensure filename is initialized

  ret = init_imu_sensor();
  if (ret) {
    LOG_ERR("Failed to initialize IMU sensor\n");
    return ret;
  }

  led_init();

  ret = bt_enable(bt_ready);
  if (ret) {
    LOG_ERR("Bluetooth init failed (err %d)", ret);
    return ret;
  }

  ret = setup_disk();
  if (ret) {
    LOG_ERR("Failed to setup disk\n");
    return ret;
  }

  /* Start in USB mass storage mode */
  k_sleep(K_MSEC(500));
  LOG_INF("The device is put in USB mass storage mode.\n");
  ret = usb_enable(NULL);
  if (ret != 0) {
    LOG_ERR("Failed to enable USB");
    return 0;
  }

  /* Main Loop */
  while (1) {
    /* State Transitions */
    if (requested_state != current_state) {
      LOG_INF("State change requested from %d to %d", current_state,
              requested_state);

      if (current_state == SYSTEM_LOGGING) {
        ret = stop_logging(&file, file_path);
        if (ret) {
          LOG_ERR("Failed to stop logging\n");
        }
      }

      if (requested_state == SYSTEM_LOGGING) {
        disable_usb_mass_storage();
        ret = start_logging(&file, file_path);
        if (ret) {
          LOG_ERR("Failed to start logging\n");
        }

        /* Calibrate the sensors */
        calibrate_accelerometer(1000); // Take 1000 samples for calibration
        calibrate_gyroscope(1000);
      }
      current_state = requested_state;
    }

    /* State Actions */
    if (current_state == SYSTEM_IDLE) {
      // "Idle state"
      led_update(BLUE_LED, true);
      k_sleep(K_MSEC(200));
      led_update(BLUE_LED, false);
      k_sleep(K_MSEC(2500));
    } else if (current_state == SYSTEM_LOGGING) {
      // "Starting Logging"
      get_imu_values(&accel_x_out, &accel_y_out, &accel_z_out, &gyro_x_out,
                     &gyro_y_out, &gyro_z_out, &magn_x_out, &magn_y_out,
                     &magn_z_out); // Get magnetometer

      size_t len = snprintf(
          (char *)write_buffer + buffer_pos, WRITE_BUFFER_SIZE - buffer_pos,
          "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", accel_x_out,
          accel_y_out, accel_z_out, gyro_x_out, gyro_y_out, gyro_z_out,
          magn_x_out, magn_y_out, magn_z_out);

      if (buffer_pos + len >= WRITE_BUFFER_SIZE) {
        flush_buffer(&file);
      }
      buffer_pos += len;

      // Periodic sync every 32 writes
      static uint32_t write_count = 0;
      if (++write_count >= 32) {
        flush_buffer(&file);
        write_count = 0;
      }
      led_log_light();
      k_sleep(K_MSEC(10));
    } else if (current_state == SYSTEM_STOP_LOGGING) {
      // "Stop Logging"
      led_update(GREEN_LED, true);
      k_sleep(K_MSEC(200));
      led_update(GREEN_LED, false);
      k_sleep(K_MSEC(2500));
    } else if (current_state == SYSTEM_OFF) {
      // "Stop Logging"
      led_update(RED_LED, true);
      k_sleep(K_MSEC(500));
      led_update(GREEN_LED, true);
      k_sleep(K_MSEC(500));
      led_update(RED_LED, false);
      k_sleep(K_MSEC(500));
      led_update(BLUE_LED, true);
      k_sleep(K_MSEC(500));
      led_update(GREEN_LED, false);
      k_sleep(K_MSEC(500));
      led_update(RED_LED, true);
      k_sleep(K_MSEC(500));
      led_update(RED_LED, false);
      led_update(BLUE_LED, false);
      sys_poweroff();
    }
  }

  return 0;
}