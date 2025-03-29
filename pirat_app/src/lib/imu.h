#ifndef IMU_H_
#define IMU_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public API */
int init_imu_sensor(void);
int read_imu_values(void);
int get_imu_values(double *accel_x_out_ptr, double *accel_y_out_ptr,
                   double *accel_z_out_ptr, double *gyro_x_out_ptr,
                   double *gyro_y_out_ptr, double *gyro_z_out_ptr,
                   double *magn_x_out_ptr, double *magn_y_out_ptr,
                   double *magn_z_out_ptr);

int calibrate_accelerometer(uint16_t num_samples);
int calibrate_gyroscope(uint16_t num_samples);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */