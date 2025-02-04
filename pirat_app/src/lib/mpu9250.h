#include <stdio.h>
int init_mpu9250_sensor(void);

int read_mpu9250_values(void);

int get_mpu9250_values(double *accel_x_out_ptr, double *accel_y_out_ptr,
                       double *accel_z_out_ptr, double *gyro_x_out_ptr,
                       double *gyro_y_out_ptr, double *gyro_z_out_ptr,
                       double *mag_x_out_ptr, double *mag_y_out_ptr,
                       double *mag_z_out_ptr);