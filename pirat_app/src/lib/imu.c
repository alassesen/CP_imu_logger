#include "imu.h"
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

#define ICM20948_NODE DT_NODELABEL(icm20948) //  DT_ALIAS(my_icm20948)
const struct device *imu_dev = DEVICE_DT_GET(ICM20948_NODE);

static struct sensor_value accel_bias[3] = {{0, 0}, {0, 0}, {0, 0}};
static struct sensor_value gyro_bias[3] = {{0, 0}, {0, 0}, {0, 0}};

// Static variables for storing sensor data
static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
static struct sensor_value magn_x_out, magn_y_out, magn_z_out;

int init_imu_sensor(void) {
  if (!imu_dev) {
    LOG_ERR("IMU device not found in devicetree!\n");
    return -ENODEV;
  }

  if (!device_is_ready(imu_dev)) {
    LOG_ERR("IMU device %s is not ready\n", imu_dev->name);
    return -ENODEV;
  }

  LOG_INF("Found IMU sensor %s\n", imu_dev->name);
  return 0;
}

int read_imu_values(void) {
  struct sensor_value accel_x, accel_y, accel_z;
  struct sensor_value gyro_x, gyro_y, gyro_z;
  struct sensor_value magn_x, magn_y, magn_z;

  if (!imu_dev) {
    LOG_ERR("IMU device not initialized!\n");
    return -ENODEV;
  }

  /* Fetch sensor data */
  if (sensor_sample_fetch(imu_dev) < 0) {
    LOG_ERR("Failed to fetch sensor sample\n");
    return -EIO;
  }

  /* Get accelerometer data */
  if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x) < 0 ||
      sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y) < 0 ||
      sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z) < 0) {
    LOG_ERR("Failed to get accelerometer data\n");
    return -EIO;
  }

  /* Get gyroscope data */
  if (sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x) < 0 ||
      sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y) < 0 ||
      sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z) < 0) {
    LOG_ERR("Failed to get gyroscope data\n");
    return -EIO;
  }
  /* Get magnetometer data */
  if (sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_X, &magn_x) < 0 ||
      sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_Y, &magn_y) < 0 ||
      sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_Z, &magn_z) < 0) {
    LOG_ERR("Failed to get magnetometer data\n");
    return -EIO;
  }

  /* Assign values to static variables */
  accel_x_out = accel_x;
  accel_y_out = accel_y;
  accel_z_out = accel_z;

  gyro_x_out = gyro_x;
  gyro_y_out = gyro_y;
  gyro_z_out = gyro_z;

  magn_x_out = magn_x;
  magn_y_out = magn_y;
  magn_z_out = magn_z;

  char accel_str[128];
  snprintf(accel_str, sizeof(accel_str), "Accelerometer: X=%f, Y=%f, Z=%f",
           sensor_value_to_double(&accel_x), sensor_value_to_double(&accel_y),
           sensor_value_to_double(&accel_z));
  LOG_INF("%s", accel_str);

  char gyro_str[128];
  snprintf(gyro_str, sizeof(gyro_str), "Gyroscope: X=%f, Y=%f, Z=%f",
           sensor_value_to_double(&gyro_x), sensor_value_to_double(&gyro_y),
           sensor_value_to_double(&gyro_z));
  LOG_INF("%s", gyro_str);

  char magn_str[128];
  snprintf(magn_str, sizeof(magn_str), "Magnetometer: X=%f, Y=%f, Z=%f",
           sensor_value_to_double(&magn_x), sensor_value_to_double(&magn_y),
           sensor_value_to_double(&magn_z));
  LOG_INF("%s", magn_str);

  return 0;
}

int get_imu_values(double *accel_x_out_ptr, double *accel_y_out_ptr,
                   double *accel_z_out_ptr, double *gyro_x_out_ptr,
                   double *gyro_y_out_ptr, double *gyro_z_out_ptr,
                   double *magn_x_out_ptr, double *magn_y_out_ptr,
                   double *magn_z_out_ptr) {
  int ret = 0;
  if (!imu_dev) {
    LOG_ERR("IMU device not initialized!\n");
    return -ENODEV;
  }

  ret = sensor_sample_fetch(imu_dev);

  if (ret == 0) {
    struct sensor_value accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
    if (ret != 0)
      *accel_x_out_ptr = -1.1111;
    else
      *accel_x_out_ptr = sensor_value_to_double(&accel_x) -
                         sensor_value_to_double(&accel_bias[0]);

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
    if (ret != 0)
      *accel_y_out_ptr = -1.1111;
    else
      *accel_y_out_ptr = sensor_value_to_double(&accel_y) -
                         sensor_value_to_double(&accel_bias[1]);

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
    if (ret != 0)
      *accel_z_out_ptr = -1.1111;
    else
      *accel_z_out_ptr = sensor_value_to_double(&accel_z) -
                         sensor_value_to_double(&accel_bias[2]);

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
    if (ret != 0)
      *gyro_x_out_ptr = -1.1111;
    else
      *gyro_x_out_ptr = sensor_value_to_double(&gyro_x) -
                        sensor_value_to_double(&gyro_bias[0]);

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
    if (ret != 0)
      *gyro_y_out_ptr = -1.1111;
    else
      *gyro_y_out_ptr = sensor_value_to_double(&gyro_y) -
                        sensor_value_to_double(&gyro_bias[1]);

    ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
    if (ret != 0)
      *gyro_z_out_ptr = -1.1111;
    else
      *gyro_z_out_ptr = sensor_value_to_double(&gyro_z) -
                        sensor_value_to_double(&gyro_bias[2]);

    /* Magnetometer */
    struct sensor_value magn_x, magn_y, magn_z;
    if (ret == 0) {
      ret = sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_X, &magn_x);
      if (ret != 0)
        *magn_x_out_ptr = -1.1111;
      else
        *magn_x_out_ptr = sensor_value_to_double(&magn_x);

      ret = sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_Y, &magn_y);
      if (ret != 0)
        *magn_y_out_ptr = -1.1111;
      else
        *magn_y_out_ptr = sensor_value_to_double(&magn_y);

      ret = sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_Z, &magn_z);
      if (ret != 0)
        *magn_z_out_ptr = -1.1111;
      else
        *magn_z_out_ptr = sensor_value_to_double(&magn_z);
    }
  }

  return 0;
}

int calibrate_accelerometer(uint16_t num_samples) {
  struct sensor_value accel_x, accel_y, accel_z;
  struct sensor_value sum[3] = {{0, 0}, {0, 0}, {0, 0}};

  if (!imu_dev) {
    LOG_ERR("IMU device not initialized!\n");
    return -ENODEV;
  }

  LOG_INF("Starting accelerometer calibration (%d samples)...\n", num_samples);

  /* Take num_samples samples and calculate the average */
  for (int i = 0; i < num_samples; i++) {
    if (sensor_sample_fetch(imu_dev) < 0) {
      LOG_ERR("Failed to fetch sensor sample during calibration\n");
      return -EIO;
    }

    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

    sum[0].val1 += accel_x.val1;
    sum[0].val2 += accel_x.val2;
    sum[1].val1 += accel_y.val1;
    sum[1].val2 += accel_y.val2;
    sum[2].val1 += accel_z.val1;
    sum[2].val2 += accel_z.val2;

    k_sleep(K_MSEC(10));
  }

  /* Calculate the average */
  accel_bias[0].val1 = sum[0].val1 / num_samples;
  accel_bias[0].val2 = sum[0].val2 / num_samples;
  accel_bias[1].val1 = sum[1].val1 / num_samples;
  accel_bias[1].val2 = sum[1].val2 / num_samples;
  accel_bias[2].val1 = sum[2].val1 / num_samples;
  accel_bias[2].val2 = sum[2].val2 / num_samples;

  /* Account for gravity on the Z axis */
  accel_bias[2].val1 -= 1;

  LOG_INF("Accelerometer biases: X=%d.%06d, Y=%d.%06d, Z=%d.%06d\n",
          accel_bias[0].val1, accel_bias[0].val2, accel_bias[1].val1,
          accel_bias[1].val2, accel_bias[2].val1, accel_bias[2].val2);

  return 0;
}

int calibrate_gyroscope(uint16_t num_samples) {
  struct sensor_value gyro_x, gyro_y, gyro_z;
  struct sensor_value sum[3] = {{0, 0}, {0, 0}, {0, 0}};

  if (!imu_dev) {
    LOG_ERR("IMU device not initialized!\n");
    return -ENODEV;
  }

  LOG_INF("Starting gyroscope calibration (%d samples)...\n", num_samples);

  /* Take num_samples samples and calculate the average */
  for (int i = 0; i < num_samples; i++) {
    if (sensor_sample_fetch(imu_dev) < 0) {
      LOG_ERR("Failed to fetch sensor sample during calibration\n");
      return -EIO;
    }

    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

    sum[0].val1 += gyro_x.val1;
    sum[0].val2 += gyro_x.val2;
    sum[1].val1 += gyro_y.val1;
    sum[1].val2 += gyro_y.val2;
    sum[2].val1 += gyro_z.val1;
    sum[2].val2 += gyro_z.val2;

    k_sleep(K_MSEC(10));
  }

  /* Calculate the average */
  gyro_bias[0].val1 = sum[0].val1 / num_samples;
  gyro_bias[0].val2 = sum[0].val2 / num_samples;
  gyro_bias[1].val1 = sum[1].val1 / num_samples;
  gyro_bias[1].val2 = sum[1].val2 / num_samples;
  gyro_bias[2].val1 = sum[2].val1 / num_samples;
  gyro_bias[2].val2 = sum[2].val2 / num_samples;

  LOG_INF("Gyroscope biases: X=%d.%06d, Y=%d.%06d, Z=%d.%06d\n",
          gyro_bias[0].val1, gyro_bias[0].val2, gyro_bias[1].val1,
          gyro_bias[1].val2, gyro_bias[2].val1, gyro_bias[2].val2);

  return 0;
}