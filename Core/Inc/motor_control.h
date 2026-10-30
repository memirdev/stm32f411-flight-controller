#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MOTOR_CONTROL_OUTPUT_MAX_US       2000U
#define MOTOR_CONTROL_IDLE_US             1175U  /* idle while armed */
#define MOTOR_CONTROL_CORRECTION_LIMIT_US 150U
#define MOTOR_CONTROL_THROTTLE_MAX_US     \
    (MOTOR_CONTROL_OUTPUT_MAX_US - MOTOR_CONTROL_CORRECTION_LIMIT_US)
#define MOTOR_CONTROL_ARM_THROTTLE_MAX_US 1200U
#define MOTOR_CONTROL_THROTTLE_RISE_STEP_US 20U
#define MOTOR_CONTROL_ARM_MAX_TILT_DEG    10.0f  /* no arming above this tilt */

#if MOTOR_CONTROL_IDLE_US >= MOTOR_CONTROL_THROTTLE_MAX_US
#error "Invalid motor throttle range"
#endif

#if MOTOR_CONTROL_THROTTLE_MAX_US >= MOTOR_CONTROL_OUTPUT_MAX_US
#error "Invalid motor output range for PID correction margin"
#endif

#if (MOTOR_CONTROL_ARM_THROTTLE_MAX_US < MOTOR_CONTROL_IDLE_US) || \
    (MOTOR_CONTROL_ARM_THROTTLE_MAX_US > MOTOR_CONTROL_THROTTLE_MAX_US)
#error "Invalid arm throttle lock range"
#endif

#if MOTOR_CONTROL_THROTTLE_RISE_STEP_US == 0U
#error "Throttle ramp step must not be zero"
#endif

typedef enum
{
    MOTOR_CONTROL_DISARMED = 0U,
    MOTOR_CONTROL_ARMED,
    MOTOR_CONTROL_BLOCKED_ESC,
    MOTOR_CONTROL_BLOCKED_IMU,
    MOTOR_CONTROL_BLOCKED_THROTTLE,
    MOTOR_CONTROL_BLOCKED_TILT
} MotorControl_State_t;

extern volatile uint8_t motor_arm_request;
extern volatile uint8_t motor_armed;
extern volatile uint16_t motor_throttle_us;
extern volatile uint16_t motor_applied_throttle_us;
extern volatile uint8_t motor_throttle_ramp_active;
extern volatile uint32_t motor_throttle_ramp_step_count;
extern volatile uint16_t motor_throttle_ramp_max_step_us;
extern volatile float motor_roll_mix_us;
extern volatile float motor_pitch_mix_us;
extern volatile float motor_yaw_mix_us;
extern volatile float motor_roll_mix_peak_us;
extern volatile float motor_pitch_mix_peak_us;
extern volatile float motor_yaw_mix_peak_us;
extern volatile uint16_t motor_roll_mix_peak_throttle_us;
extern volatile uint32_t motor_mix_peak_session_count;
extern volatile uint32_t motor_mix_shifted_count;
extern volatile uint32_t motor_mix_scaled_count;
extern volatile float motor_mix_shift_peak_us;
extern volatile MotorControl_State_t motor_control_state;

uint16_t MotorControl_ThrottleFromNormalized(float throttle_normalized);

void MotorControl_Init(void);
void MotorControl_ForceDisarm(void);
void MotorControl_Update(HAL_StatusTypeDef imu_status,
                         uint8_t imu_ready,
                         float roll_deg,
                         float pitch_deg,
                         float roll_pid_us,
                         float pitch_pid_us,
                         float yaw_pid_us);

#ifdef __cplusplus
}
#endif

#endif
