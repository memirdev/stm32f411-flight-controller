#include "imu_filter.h"

#include <math.h>
#include <stddef.h>

#define IMU_PI_F 3.14159265358979323846f
#define IMU_RAD_TO_DEG (180.0f / IMU_PI_F)

static float IMU_Filter_LowPassAlpha(float cutoff_hz, float dt_s)
{
    float rc_s;

    if (cutoff_hz <= 0.0f)
    {
        return 1.0f;
    }

    rc_s = 1.0f / (2.0f * IMU_PI_F * cutoff_hz);
    return dt_s / (rc_s + dt_s);
}

static float IMU_Filter_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void IMU_Filter_Init(IMU_Filter_t *filter,
                     float accel_cutoff_hz,
                     float gyro_cutoff_hz,
                     float complementary_tau_s)
{
    if (filter == NULL)
    {
        return;
    }

    filter->gyro_bias_x_dps = 0.0f;
    filter->gyro_bias_y_dps = 0.0f;
    filter->gyro_bias_z_dps = 0.0f;

    filter->accel_x_g = 0.0f;
    filter->accel_y_g = 0.0f;
    filter->accel_z_g = 0.0f;
    filter->gyro_x_dps = 0.0f;
    filter->gyro_y_dps = 0.0f;
    filter->gyro_z_dps = 0.0f;
    filter->roll_deg = 0.0f;
    filter->pitch_deg = 0.0f;

    filter->accel_cutoff_hz = accel_cutoff_hz;
    filter->gyro_cutoff_hz = gyro_cutoff_hz;
    filter->complementary_tau_s = complementary_tau_s;
    filter->initialized = 0U;
    filter->accel_weight = 1.0f;
    filter->accel_reject_count = 0U;
}

void IMU_Filter_SetGyroBias(IMU_Filter_t *filter,
                            float bias_x_dps,
                            float bias_y_dps,
                            float bias_z_dps)
{
    if (filter == NULL)
    {
        return;
    }

    filter->gyro_bias_x_dps = bias_x_dps;
    filter->gyro_bias_y_dps = bias_y_dps;
    filter->gyro_bias_z_dps = bias_z_dps;
}

void IMU_Filter_Update(IMU_Filter_t *filter,
                       const MPU6500_Data_t *data,
                       float dt_s)
{
    float accel_alpha;
    float gyro_alpha;
    float complementary_alpha;
    float corrected_gyro_x;
    float corrected_gyro_y;
    float corrected_gyro_z;
    float accel_roll_deg;
    float accel_pitch_deg;
    float roll_prediction;
    float pitch_prediction;
    float roll_error;
    float pitch_error;
    float accel_norm_error_g;
    float accel_weight;

    if ((filter == NULL) || (data == NULL) ||
        (dt_s <= 0.0f) || (dt_s > 0.1f))
    {
        return;
    }

    /* Sensor axes to body axes (module: X forward, Y left, Z up). */
    corrected_gyro_x = data->gyro_x_dps - filter->gyro_bias_x_dps;
    corrected_gyro_y = -(data->gyro_y_dps - filter->gyro_bias_y_dps);
    corrected_gyro_z = -(data->gyro_z_dps - filter->gyro_bias_z_dps);

    if (filter->initialized == 0U)
    {
        filter->accel_x_g = data->accel_x_g;
        filter->accel_y_g = data->accel_y_g;
        filter->accel_z_g = data->accel_z_g;
        filter->gyro_x_dps = corrected_gyro_x;
        filter->gyro_y_dps = corrected_gyro_y;
        filter->gyro_z_dps = corrected_gyro_z;

        filter->roll_deg = atan2f(filter->accel_y_g,
                                  filter->accel_z_g) * IMU_RAD_TO_DEG;
        filter->pitch_deg = atan2f(filter->accel_x_g,
                                   sqrtf((filter->accel_y_g * filter->accel_y_g) +
                                         (filter->accel_z_g * filter->accel_z_g))) *
                            IMU_RAD_TO_DEG;
        filter->initialized = 1U;
        return;
    }

    accel_alpha = IMU_Filter_LowPassAlpha(filter->accel_cutoff_hz, dt_s);
    gyro_alpha = IMU_Filter_LowPassAlpha(filter->gyro_cutoff_hz, dt_s);

    filter->accel_x_g += accel_alpha * (data->accel_x_g - filter->accel_x_g);
    filter->accel_y_g += accel_alpha * (data->accel_y_g - filter->accel_y_g);
    filter->accel_z_g += accel_alpha * (data->accel_z_g - filter->accel_z_g);

    filter->gyro_x_dps += gyro_alpha * (corrected_gyro_x - filter->gyro_x_dps);
    filter->gyro_y_dps += gyro_alpha * (corrected_gyro_y - filter->gyro_y_dps);
    filter->gyro_z_dps += gyro_alpha * (corrected_gyro_z - filter->gyro_z_dps);

    accel_roll_deg = atan2f(filter->accel_y_g,
                            filter->accel_z_g) * IMU_RAD_TO_DEG;
    accel_pitch_deg = atan2f(filter->accel_x_g,
                             sqrtf((filter->accel_y_g * filter->accel_y_g) +
                                   (filter->accel_z_g * filter->accel_z_g))) *
                      IMU_RAD_TO_DEG;

    accel_norm_error_g = fabsf(sqrtf((filter->accel_x_g * filter->accel_x_g) +
                                     (filter->accel_y_g * filter->accel_y_g) +
                                     (filter->accel_z_g * filter->accel_z_g)) -
                               1.0f);
    if (accel_norm_error_g <= IMU_ACCEL_TRUST_FULL_G)
    {
        accel_weight = 1.0f;
    }
    else if (accel_norm_error_g >= IMU_ACCEL_TRUST_ZERO_G)
    {
        accel_weight = 0.0f;
    }
    else
    {
        accel_weight = (IMU_ACCEL_TRUST_ZERO_G - accel_norm_error_g) /
                       (IMU_ACCEL_TRUST_ZERO_G - IMU_ACCEL_TRUST_FULL_G);
    }
    filter->accel_weight = accel_weight;
    if (accel_weight < 0.5f)
    {
        filter->accel_reject_count++;
    }

    /* Complementary filter: the angle is integrated from the gyro and slowly pulled toward the accel angle. */
    complementary_alpha = filter->complementary_tau_s /
                          (filter->complementary_tau_s + dt_s);

    roll_prediction = IMU_Filter_Wrap180(filter->roll_deg +
                                         (filter->gyro_x_dps * dt_s));
    pitch_prediction = IMU_Filter_Wrap180(filter->pitch_deg +
                                          (filter->gyro_y_dps * dt_s));

    roll_error = IMU_Filter_Wrap180(accel_roll_deg - roll_prediction);
    pitch_error = IMU_Filter_Wrap180(accel_pitch_deg - pitch_prediction);

    filter->roll_deg = IMU_Filter_Wrap180(roll_prediction +
                                           ((1.0f - complementary_alpha) *
                                            accel_weight * roll_error));
    filter->pitch_deg = IMU_Filter_Wrap180(pitch_prediction +
                                            ((1.0f - complementary_alpha) *
                                             accel_weight * pitch_error));
}
