#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6500.h"
#include <stdint.h>

/* The accelerometer is used for the angle only while |a| ~ 1 g (in free fall,
 * sudden throttle and hard maneuvers the accel-derived angle is wrong). */
#define IMU_ACCEL_TRUST_FULL_G      0.15f   /* full trust up to this deviation */
#define IMU_ACCEL_TRUST_ZERO_G      0.40f   /* gyro only beyond this deviation */

/* Axes: roll + right side down, pitch + nose up, yaw + clockwise from above. */
typedef struct
{
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float roll_deg;
    float pitch_deg;

    float accel_cutoff_hz;
    float gyro_cutoff_hz;
    float complementary_tau_s;
    uint8_t initialized;

    float accel_weight;             /* accelerometer trust 0..1 (diagnostic) */
    uint32_t accel_reject_count;    /* loops with trust below 0.5 */
} IMU_Filter_t;

void IMU_Filter_Init(IMU_Filter_t *filter,
                     float accel_cutoff_hz,
                     float gyro_cutoff_hz,
                     float complementary_tau_s);

void IMU_Filter_SetGyroBias(IMU_Filter_t *filter,
                            float bias_x_dps,
                            float bias_y_dps,
                            float bias_z_dps);

void IMU_Filter_Update(IMU_Filter_t *filter,
                       const MPU6500_Data_t *data,
                       float dt_s);

#ifdef __cplusplus
}
#endif

#endif
