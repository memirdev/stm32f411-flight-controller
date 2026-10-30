#include "flight_diagnostics.h"

#include <math.h>
#include <stddef.h>

volatile uint32_t flight_diag_reset_request = 0U;

volatile uint8_t flight_diag_collecting_motor_armed = 0U;
volatile uint8_t flight_diag_last_window_motor_armed = 0U;
volatile uint16_t flight_diag_samples_in_window = 0U;
volatile uint32_t flight_diag_window_count = 0U;
volatile uint32_t flight_diag_invalid_dt_count = 0U;
volatile uint32_t flight_diag_loop_overrun_count = 0U;
volatile uint16_t flight_diag_loop_overruns_last_window = 0U;

volatile float flight_diag_loop_period_current_ms = 0.0f;
volatile float flight_diag_loop_period_avg_ms = 0.0f;
volatile float flight_diag_loop_period_min_ms = 0.0f;
volatile float flight_diag_loop_period_max_ms = 0.0f;
volatile float flight_diag_loop_frequency_hz = 0.0f;

volatile float flight_diag_accel_norm_current_g = 0.0f;
volatile float flight_diag_accel_norm_mean_g = 0.0f;
volatile float flight_diag_accel_vibration_rms_g = 0.0f;
volatile float flight_diag_accel_vibration_peak_g = 0.0f;
volatile float flight_diag_gyro_magnitude_current_dps = 0.0f;
volatile float flight_diag_gyro_vibration_rms_dps = 0.0f;
volatile float flight_diag_gyro_vibration_peak_dps = 0.0f;

volatile uint8_t flight_diag_armed_window_valid = 0U;
volatile uint32_t flight_diag_armed_window_count = 0U;
volatile uint32_t flight_diag_armed_session_count = 0U;
volatile float flight_diag_armed_last_accel_vibration_rms_g = 0.0f;
volatile float flight_diag_armed_last_accel_vibration_peak_g = 0.0f;
volatile float flight_diag_armed_last_gyro_vibration_rms_dps = 0.0f;
volatile float flight_diag_armed_last_gyro_vibration_peak_dps = 0.0f;
volatile float flight_diag_armed_peak_accel_vibration_rms_g = 0.0f;
volatile float flight_diag_armed_peak_accel_vibration_peak_g = 0.0f;
volatile float flight_diag_armed_peak_gyro_vibration_rms_dps = 0.0f;
volatile float flight_diag_armed_peak_gyro_vibration_peak_dps = 0.0f;

static uint32_t flight_diag_last_reset_request = 0U;
static float flight_diag_dt_sum_s = 0.0f;
static float flight_diag_dt_min_s = 0.0f;
static float flight_diag_dt_max_s = 0.0f;
static float flight_diag_accel_norm_sum = 0.0f;
static float flight_diag_accel_norm_sq_sum = 0.0f;
static float flight_diag_accel_norm_min = 0.0f;
static float flight_diag_accel_norm_max = 0.0f;
static float flight_diag_gyro_axis_sum[3] = {0.0f, 0.0f, 0.0f};
static float flight_diag_gyro_axis_sq_sum[3] = {0.0f, 0.0f, 0.0f};
static float flight_diag_gyro_axis_min[3] = {0.0f, 0.0f, 0.0f};
static float flight_diag_gyro_axis_max[3] = {0.0f, 0.0f, 0.0f};
static uint16_t flight_diag_window_overruns = 0U;

static float FlightDiagnostics_NonNegative(float value)
{
    return (value > 0.0f) ? value : 0.0f;
}

static void FlightDiagnostics_ResetWindow(uint8_t motor_armed)
{
    flight_diag_collecting_motor_armed = (motor_armed != 0U) ? 1U : 0U;
    flight_diag_samples_in_window = 0U;
    flight_diag_dt_sum_s = 0.0f;
    flight_diag_dt_min_s = 0.0f;
    flight_diag_dt_max_s = 0.0f;
    flight_diag_accel_norm_sum = 0.0f;
    flight_diag_accel_norm_sq_sum = 0.0f;
    flight_diag_accel_norm_min = 0.0f;
    flight_diag_accel_norm_max = 0.0f;
    flight_diag_gyro_axis_sum[0] = 0.0f;
    flight_diag_gyro_axis_sum[1] = 0.0f;
    flight_diag_gyro_axis_sum[2] = 0.0f;
    flight_diag_gyro_axis_sq_sum[0] = 0.0f;
    flight_diag_gyro_axis_sq_sum[1] = 0.0f;
    flight_diag_gyro_axis_sq_sum[2] = 0.0f;
    flight_diag_gyro_axis_min[0] = 0.0f;
    flight_diag_gyro_axis_min[1] = 0.0f;
    flight_diag_gyro_axis_min[2] = 0.0f;
    flight_diag_gyro_axis_max[0] = 0.0f;
    flight_diag_gyro_axis_max[1] = 0.0f;
    flight_diag_gyro_axis_max[2] = 0.0f;
    flight_diag_window_overruns = 0U;
}

static void FlightDiagnostics_ResetArmedSession(void)
{
    flight_diag_armed_window_valid = 0U;
    flight_diag_armed_window_count = 0U;
    flight_diag_armed_last_accel_vibration_rms_g = 0.0f;
    flight_diag_armed_last_accel_vibration_peak_g = 0.0f;
    flight_diag_armed_last_gyro_vibration_rms_dps = 0.0f;
    flight_diag_armed_last_gyro_vibration_peak_dps = 0.0f;
    flight_diag_armed_peak_accel_vibration_rms_g = 0.0f;
    flight_diag_armed_peak_accel_vibration_peak_g = 0.0f;
    flight_diag_armed_peak_gyro_vibration_rms_dps = 0.0f;
    flight_diag_armed_peak_gyro_vibration_peak_dps = 0.0f;
}

void FlightDiagnostics_Init(void)
{
    flight_diag_reset_request = 0U;
    flight_diag_last_reset_request = 0U;
    flight_diag_last_window_motor_armed = 0U;
    flight_diag_window_count = 0U;
    flight_diag_invalid_dt_count = 0U;
    flight_diag_loop_overrun_count = 0U;
    flight_diag_loop_overruns_last_window = 0U;
    flight_diag_loop_period_current_ms = 0.0f;
    flight_diag_loop_period_avg_ms = 0.0f;
    flight_diag_loop_period_min_ms = 0.0f;
    flight_diag_loop_period_max_ms = 0.0f;
    flight_diag_loop_frequency_hz = 0.0f;
    flight_diag_accel_norm_current_g = 0.0f;
    flight_diag_accel_norm_mean_g = 0.0f;
    flight_diag_accel_vibration_rms_g = 0.0f;
    flight_diag_accel_vibration_peak_g = 0.0f;
    flight_diag_gyro_magnitude_current_dps = 0.0f;
    flight_diag_gyro_vibration_rms_dps = 0.0f;
    flight_diag_gyro_vibration_peak_dps = 0.0f;
    flight_diag_armed_session_count = 0U;
    FlightDiagnostics_ResetArmedSession();
    FlightDiagnostics_ResetWindow(0U);
}

void FlightDiagnostics_Update(float dt_s,
                              const MPU6500_Data_t *imu_data,
                              uint8_t motor_armed)
{
    float accel_norm_g;
    float gyro_magnitude_dps;
    float gyro_axis[3];
    uint8_t axis;
    uint8_t armed_snapshot = (motor_armed != 0U) ? 1U : 0U;

    if (imu_data == NULL)
    {
        return;
    }

    if (flight_diag_last_reset_request != flight_diag_reset_request)
    {
        flight_diag_last_reset_request = flight_diag_reset_request;
        FlightDiagnostics_ResetArmedSession();
        FlightDiagnostics_ResetWindow(armed_snapshot);
    }
    else if (flight_diag_collecting_motor_armed != armed_snapshot)
    {
        if (armed_snapshot != 0U)
        {
            FlightDiagnostics_ResetArmedSession();
            flight_diag_armed_session_count++;
        }
        FlightDiagnostics_ResetWindow(armed_snapshot);
    }

    if ((dt_s <= 0.0f) || (dt_s > 0.1f))
    {
        flight_diag_invalid_dt_count++;
        return;
    }

    accel_norm_g = sqrtf((imu_data->accel_x_g * imu_data->accel_x_g) +
                         (imu_data->accel_y_g * imu_data->accel_y_g) +
                         (imu_data->accel_z_g * imu_data->accel_z_g));
    gyro_axis[0] = imu_data->gyro_x_dps;
    gyro_axis[1] = imu_data->gyro_y_dps;
    gyro_axis[2] = imu_data->gyro_z_dps;
    gyro_magnitude_dps =
        sqrtf((gyro_axis[0] * gyro_axis[0]) +
              (gyro_axis[1] * gyro_axis[1]) +
              (gyro_axis[2] * gyro_axis[2]));

    flight_diag_loop_period_current_ms = dt_s * 1000.0f;
    flight_diag_accel_norm_current_g = accel_norm_g;
    flight_diag_gyro_magnitude_current_dps = gyro_magnitude_dps;

    if (flight_diag_samples_in_window == 0U)
    {
        flight_diag_dt_min_s = dt_s;
        flight_diag_dt_max_s = dt_s;
        flight_diag_accel_norm_min = accel_norm_g;
        flight_diag_accel_norm_max = accel_norm_g;
        for (axis = 0U; axis < 3U; axis++)
        {
            flight_diag_gyro_axis_min[axis] = gyro_axis[axis];
            flight_diag_gyro_axis_max[axis] = gyro_axis[axis];
        }
    }
    else
    {
        if (dt_s < flight_diag_dt_min_s)
        {
            flight_diag_dt_min_s = dt_s;
        }
        if (dt_s > flight_diag_dt_max_s)
        {
            flight_diag_dt_max_s = dt_s;
        }
        if (accel_norm_g < flight_diag_accel_norm_min)
        {
            flight_diag_accel_norm_min = accel_norm_g;
        }
        if (accel_norm_g > flight_diag_accel_norm_max)
        {
            flight_diag_accel_norm_max = accel_norm_g;
        }
        for (axis = 0U; axis < 3U; axis++)
        {
            if (gyro_axis[axis] < flight_diag_gyro_axis_min[axis])
            {
                flight_diag_gyro_axis_min[axis] = gyro_axis[axis];
            }
            if (gyro_axis[axis] > flight_diag_gyro_axis_max[axis])
            {
                flight_diag_gyro_axis_max[axis] = gyro_axis[axis];
            }
        }
    }

    flight_diag_dt_sum_s += dt_s;
    flight_diag_accel_norm_sum += accel_norm_g;
    flight_diag_accel_norm_sq_sum += accel_norm_g * accel_norm_g;
    for (axis = 0U; axis < 3U; axis++)
    {
        flight_diag_gyro_axis_sum[axis] += gyro_axis[axis];
        flight_diag_gyro_axis_sq_sum[axis] += gyro_axis[axis] * gyro_axis[axis];
    }
    if (flight_diag_loop_period_current_ms > FLIGHT_DIAG_LOOP_OVERRUN_MS)
    {
        flight_diag_loop_overrun_count++;
        flight_diag_window_overruns++;
    }

    flight_diag_samples_in_window++;
    if (flight_diag_samples_in_window >= FLIGHT_DIAG_WINDOW_SAMPLES)
    {
        float sample_count = (float)FLIGHT_DIAG_WINDOW_SAMPLES;
        float accel_variance;
        float gyro_variance_sum = 0.0f;
        float gyro_half_range_sq_sum = 0.0f;

        flight_diag_loop_period_avg_ms =
            (flight_diag_dt_sum_s / sample_count) * 1000.0f;
        flight_diag_loop_period_min_ms = flight_diag_dt_min_s * 1000.0f;
        flight_diag_loop_period_max_ms = flight_diag_dt_max_s * 1000.0f;
        flight_diag_loop_frequency_hz =
            (flight_diag_dt_sum_s > 0.0f)
                ? (sample_count / flight_diag_dt_sum_s)
                : 0.0f;
        flight_diag_accel_norm_mean_g =
            flight_diag_accel_norm_sum / sample_count;
        accel_variance =
            (flight_diag_accel_norm_sq_sum / sample_count) -
            (flight_diag_accel_norm_mean_g * flight_diag_accel_norm_mean_g);
        flight_diag_accel_vibration_rms_g =
            sqrtf(FlightDiagnostics_NonNegative(accel_variance));
        flight_diag_accel_vibration_peak_g =
            (flight_diag_accel_norm_max - flight_diag_accel_norm_min) * 0.5f;

        for (axis = 0U; axis < 3U; axis++)
        {
            float axis_mean = flight_diag_gyro_axis_sum[axis] / sample_count;
            float axis_variance =
                (flight_diag_gyro_axis_sq_sum[axis] / sample_count) -
                (axis_mean * axis_mean);
            float axis_half_range =
                (flight_diag_gyro_axis_max[axis] -
                 flight_diag_gyro_axis_min[axis]) * 0.5f;

            gyro_variance_sum += FlightDiagnostics_NonNegative(axis_variance);
            gyro_half_range_sq_sum += axis_half_range * axis_half_range;
        }
        flight_diag_gyro_vibration_rms_dps = sqrtf(gyro_variance_sum);
        flight_diag_gyro_vibration_peak_dps =
            sqrtf(gyro_half_range_sq_sum);
        flight_diag_loop_overruns_last_window = flight_diag_window_overruns;
        flight_diag_last_window_motor_armed = armed_snapshot;
        if (armed_snapshot != 0U)
        {
            flight_diag_armed_last_accel_vibration_rms_g =
                flight_diag_accel_vibration_rms_g;
            flight_diag_armed_last_accel_vibration_peak_g =
                flight_diag_accel_vibration_peak_g;
            flight_diag_armed_last_gyro_vibration_rms_dps =
                flight_diag_gyro_vibration_rms_dps;
            flight_diag_armed_last_gyro_vibration_peak_dps =
                flight_diag_gyro_vibration_peak_dps;

            if (flight_diag_accel_vibration_rms_g >
                flight_diag_armed_peak_accel_vibration_rms_g)
            {
                flight_diag_armed_peak_accel_vibration_rms_g =
                    flight_diag_accel_vibration_rms_g;
            }
            if (flight_diag_accel_vibration_peak_g >
                flight_diag_armed_peak_accel_vibration_peak_g)
            {
                flight_diag_armed_peak_accel_vibration_peak_g =
                    flight_diag_accel_vibration_peak_g;
            }
            if (flight_diag_gyro_vibration_rms_dps >
                flight_diag_armed_peak_gyro_vibration_rms_dps)
            {
                flight_diag_armed_peak_gyro_vibration_rms_dps =
                    flight_diag_gyro_vibration_rms_dps;
            }
            if (flight_diag_gyro_vibration_peak_dps >
                flight_diag_armed_peak_gyro_vibration_peak_dps)
            {
                flight_diag_armed_peak_gyro_vibration_peak_dps =
                    flight_diag_gyro_vibration_peak_dps;
            }

            flight_diag_armed_window_count++;
            flight_diag_armed_window_valid = 1U;
        }
        flight_diag_window_count++;
        FlightDiagnostics_ResetWindow(armed_snapshot);
    }
}
