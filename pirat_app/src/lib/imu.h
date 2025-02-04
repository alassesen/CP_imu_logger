#include <stdio.h>
extern int print_samples;
int init_imu_sensor(void);
int read_imu_values(void);
int get_imu_values(double *accel_x_out_ptr, double *accel_y_out_ptr,
                   double *accel_z_out_ptr, double *gyro_x_out_ptr,
                   double *gyro_y_out_ptr, double *gyro_z_out_ptr);