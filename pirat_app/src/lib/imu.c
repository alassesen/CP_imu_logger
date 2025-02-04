#include "imu.h"
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

int print_samples;
static char out_str[64];
static struct sensor_value odr_attr;
const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev,
                                    const struct sensor_trigger *trig) {
  static struct sensor_value accel_x, accel_y, accel_z;
  static struct sensor_value gyro_x, gyro_y, gyro_z;

  sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
  sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
  sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
  sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

  /* lsm6dsl gyro */
  sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
  sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
  sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
  sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

  accel_x_out = accel_x;
  accel_y_out = accel_y;
  accel_z_out = accel_z;

  gyro_x_out = gyro_x;
  gyro_y_out = gyro_y;
  gyro_z_out = gyro_z;
}
#endif

int init_imu_sensor(void) {
  if (!device_is_ready(lsm6dsl_dev)) {
    printk("sensor: device not ready.\n");
    return -1;
  }

  /* set accel/gyro sampling frequency to 104 Hz */
  odr_attr.val1 = 104;
  odr_attr.val2 = 0;

  if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
                      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
    printk("Cannot set sampling frequency for accelerometer.\n");
    return -1;
  }

  if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
                      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
    printk("Cannot set sampling frequency for gyro.\n");
    return -1;
  }

#ifdef CONFIG_LSM6DSL_TRIGGER
  struct sensor_trigger trig;

  trig.type = SENSOR_TRIG_DATA_READY;
  trig.chan = SENSOR_CHAN_ACCEL_XYZ;

  if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
    printk("Could not set sensor type and channel\n");
    return 0;
  }
#endif

  if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
    printk("Sensor sample update error\n");
    return -1;
  }
  return 0;
}

int read_imu_values(void) {
  printk("\0033\014");
  printf("LSM6DSL sensor samples:\n\n");

  /* lsm6dsl accel */
  sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
          sensor_value_to_double(&accel_x_out),
          sensor_value_to_double(&accel_y_out),
          sensor_value_to_double(&accel_z_out));
  printk("%s\n", out_str);

  /* lsm6dsl gyro */
  sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
          sensor_value_to_double(&gyro_x_out),
          sensor_value_to_double(&gyro_y_out),
          sensor_value_to_double(&gyro_z_out));
  printk("%s\n", out_str);
  return 0;
}

int get_imu_values(double *accel_x_out_ptr, double *accel_y_out_ptr,
                   double *accel_z_out_ptr, double *gyro_x_out_ptr,
                   double *gyro_y_out_ptr, double *gyro_z_out_ptr) {
  static struct sensor_value accel_x, accel_y, accel_z;
  static struct sensor_value gyro_x, gyro_y, gyro_z;
  int ret = 0;
  /* lsm6dsl accel */
  ret = sensor_sample_fetch_chan(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ);
  if (ret == 0) {
    ret = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
    if (ret != 0)
      *accel_x_out_ptr = -1.1111;
    else
      *accel_x_out_ptr = sensor_value_to_double(&accel_x);

    ret = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
    if (ret != 0)
      *accel_y_out_ptr = -1.1111;
    else
      *accel_y_out_ptr = sensor_value_to_double(&accel_y);

    ret = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
    if (ret != 0)
      *accel_z_out_ptr = -1.1111;
    else
      *accel_z_out_ptr = sensor_value_to_double(&accel_z);
  }

  /* lsm6dsl gyro */
  ret = sensor_sample_fetch_chan(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ);
  if (ret == 0) {
    ret = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
    if (ret != 0)
      *gyro_x_out_ptr = -1.1111;
    else
      *gyro_x_out_ptr = sensor_value_to_double(&gyro_x);

    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
    if (ret != 0)
      *gyro_y_out_ptr = -1.1111;
    else
      *gyro_y_out_ptr = sensor_value_to_double(&gyro_y);

    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
    if (ret != 0)
      *gyro_z_out_ptr = -1.1111;
    else
      *gyro_z_out_ptr = sensor_value_to_double(&gyro_z);
  }

  return 0;
}