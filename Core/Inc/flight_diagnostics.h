#ifndef FLIGHT_DIAGNOSTICS_H
#define FLIGHT_DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6500.h"
#include <stdint.h>

/* Diagnostics only: loop timing and vibration statistics. */
#define FLIGHT_DIAG_WINDOW_SAMPLES       100U
#define FLIGHT_DIAG_LOOP_OVERRUN_MS      15.0f

extern volatile uint32_t flight_diag_reset_request;

extern volatile uint8_t flight_diag_collecting_motor_armed;
extern volatile uint8_t flight_diag_last_window_motor_armed;
extern volatile uint16_t flight_diag_samples_in_window;
extern volatile uint32_t flight_diag_window_count;
extern volatile uint32_t flight_diag_invalid_dt_count;
extern volatile uint32_t flight_diag_loop_overrun_count;
extern volatile uint16_t flight_diag_loop_overruns_last_window;

extern volatile float flight_diag_loop_period_current_ms;
extern volatile float flight_diag_loop_period_avg_ms;
extern volatile float flight_diag_loop_period_min_ms;
extern volatile float flight_diag_loop_period_max_ms;
extern volatile float flight_diag_loop_frequency_hz;

extern volatile float flight_diag_accel_norm_current_g;
extern volatile float flight_diag_accel_norm_mean_g;
extern volatile float flight_diag_accel_vibration_rms_g;
extern volatile float flight_diag_accel_vibration_peak_g;
extern volatile float flight_diag_gyro_magnitude_current_dps;
extern volatile float flight_diag_gyro_vibration_rms_dps;
extern volatile float flight_diag_gyro_vibration_peak_dps;

extern volatile uint8_t flight_diag_armed_window_valid;
extern volatile uint32_t flight_diag_armed_window_count;
extern volatile uint32_t flight_diag_armed_session_count;
extern volatile float flight_diag_armed_last_accel_vibration_rms_g;
extern volatile float flight_diag_armed_last_accel_vibration_peak_g;
extern volatile float flight_diag_armed_last_gyro_vibration_rms_dps;
extern volatile float flight_diag_armed_last_gyro_vibration_peak_dps;
extern volatile float flight_diag_armed_peak_accel_vibration_rms_g;
extern volatile float flight_diag_armed_peak_accel_vibration_peak_g;
extern volatile float flight_diag_armed_peak_gyro_vibration_rms_dps;
extern volatile float flight_diag_armed_peak_gyro_vibration_peak_dps;

void FlightDiagnostics_Init(void);
void FlightDiagnostics_Update(float dt_s,
                              const MPU6500_Data_t *imu_data,
                              uint8_t motor_armed);

#ifdef __cplusplus
}
#endif

#endif
