#include "motor_control.h"

#include "esc_pwm.h"

volatile uint8_t motor_arm_request = 0U;
volatile uint8_t motor_armed = 0U;
volatile uint16_t motor_throttle_us = MOTOR_CONTROL_IDLE_US;
volatile uint16_t motor_applied_throttle_us = MOTOR_CONTROL_IDLE_US;
volatile uint8_t motor_throttle_ramp_active = 0U;
volatile uint32_t motor_throttle_ramp_step_count = 0U;
volatile uint16_t motor_throttle_ramp_max_step_us = 0U;
volatile float motor_roll_mix_us = 0.0f;
volatile float motor_pitch_mix_us = 0.0f;
volatile float motor_yaw_mix_us = 0.0f;
volatile float motor_roll_mix_peak_us = 0.0f;
volatile float motor_pitch_mix_peak_us = 0.0f;
volatile float motor_yaw_mix_peak_us = 0.0f;
volatile uint16_t motor_roll_mix_peak_throttle_us = MOTOR_CONTROL_IDLE_US;
volatile uint32_t motor_mix_peak_session_count = 0U;
volatile uint32_t motor_mix_shifted_count = 0U;
volatile uint32_t motor_mix_scaled_count = 0U;
volatile float motor_mix_shift_peak_us = 0.0f;
volatile MotorControl_State_t motor_control_state = MOTOR_CONTROL_DISARMED;

static float MotorControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t MotorControl_IsFinite(float value)
{
    return ((value == value) &&
            (value <= 3.402823466e+38f) &&
            (value >= -3.402823466e+38f)) ? 1U : 0U;
}

static float MotorControl_ClampCorrection(float correction_us)
{
    const float correction_limit_us =
        (float)MOTOR_CONTROL_CORRECTION_LIMIT_US;

    if (correction_us > correction_limit_us)
    {
        return correction_limit_us;
    }
    if (correction_us < -correction_limit_us)
    {
        return -correction_limit_us;
    }
    return correction_us;
}

static uint16_t MotorControl_ClampThrottle(uint16_t throttle_us)
{
    if (throttle_us < MOTOR_CONTROL_IDLE_US)
    {
        return MOTOR_CONTROL_IDLE_US;
    }
    if (throttle_us > MOTOR_CONTROL_THROTTLE_MAX_US)
    {
        return MOTOR_CONTROL_THROTTLE_MAX_US;
    }
    return throttle_us;
}

static uint16_t MotorControl_ClampOutput(float output_us)
{
    if (output_us < (float)MOTOR_CONTROL_IDLE_US)
    {
        return MOTOR_CONTROL_IDLE_US;
    }
    if (output_us > (float)MOTOR_CONTROL_OUTPUT_MAX_US)
    {
        return MOTOR_CONTROL_OUTPUT_MAX_US;
    }
    return (uint16_t)(output_us + 0.5f);
}

static uint16_t MotorControl_ApplyThrottleRamp(uint16_t requested_us)
{
    uint16_t previous_applied_us = motor_applied_throttle_us;
    uint16_t applied_step_us;

    if (requested_us <= motor_applied_throttle_us)
    {
        motor_applied_throttle_us = requested_us;
        motor_throttle_ramp_active = 0U;
    }
    else
    {
        if (motor_applied_throttle_us == MOTOR_CONTROL_IDLE_US)
        {
            motor_throttle_ramp_step_count = 0U;
            motor_throttle_ramp_max_step_us = 0U;
        }

        if (motor_throttle_ramp_active == 0U)
        {
            motor_throttle_ramp_active = 1U;
        }

        if ((uint32_t)(requested_us - motor_applied_throttle_us) >
            MOTOR_CONTROL_THROTTLE_RISE_STEP_US)
        {
            motor_applied_throttle_us +=
                MOTOR_CONTROL_THROTTLE_RISE_STEP_US;
        }
        else
        {
            motor_applied_throttle_us = requested_us;
        }

        motor_throttle_ramp_step_count++;
        applied_step_us = motor_applied_throttle_us - previous_applied_us;
        if (applied_step_us > motor_throttle_ramp_max_step_us)
        {
            motor_throttle_ramp_max_step_us = applied_step_us;
        }
        if (motor_applied_throttle_us >= requested_us)
        {
            motor_throttle_ramp_active = 0U;
        }
    }

    return motor_applied_throttle_us;
}

static void MotorControl_Block(MotorControl_State_t state)
{
    motor_arm_request = 0U;
    motor_armed = 0U;
    motor_applied_throttle_us = MOTOR_CONTROL_IDLE_US;
    motor_throttle_ramp_active = 0U;
    motor_roll_mix_us = 0.0f;
    motor_pitch_mix_us = 0.0f;
    motor_yaw_mix_us = 0.0f;
    motor_control_state = state;
    ESC_PWM_SetMinimum();
}

uint16_t MotorControl_ThrottleFromNormalized(float throttle_normalized)
{
    float pulse_us;

    if ((throttle_normalized != throttle_normalized) ||
        (throttle_normalized < 0.0f))
    {
        throttle_normalized = 0.0f;
    }
    else if (throttle_normalized > 1.0f)
    {
        throttle_normalized = 1.0f;
    }

    pulse_us = (float)MOTOR_CONTROL_IDLE_US +
               (throttle_normalized *
                (float)(MOTOR_CONTROL_THROTTLE_MAX_US -
                        MOTOR_CONTROL_IDLE_US));
    return (uint16_t)(pulse_us + 0.5f);
}

void MotorControl_Init(void)
{
    motor_throttle_us = MOTOR_CONTROL_IDLE_US;
    MotorControl_ForceDisarm();
}

void MotorControl_ForceDisarm(void)
{
    MotorControl_Block(MOTOR_CONTROL_DISARMED);
}

static void MotorControl_NormalizeMix(float *motor_us, float base_us)
{
    const float floor_us = (float)MOTOR_CONTROL_IDLE_US;
    const float ceiling_us = (float)MOTOR_CONTROL_OUTPUT_MAX_US;
    const float window_us = ceiling_us - floor_us;
    float min_us;
    float max_us;
    float span_us;
    float shift_us;
    uint8_t index;

    min_us = motor_us[0];
    max_us = motor_us[0];
    for (index = 1U; index < 4U; index++)
    {
        if (motor_us[index] < min_us)
        {
            min_us = motor_us[index];
        }
        if (motor_us[index] > max_us)
        {
            max_us = motor_us[index];
        }
    }

    span_us = max_us - min_us;
    if (span_us > window_us)
    {
        float scale = window_us / span_us;

        for (index = 0U; index < 4U; index++)
        {
            motor_us[index] = base_us + ((motor_us[index] - base_us) * scale);
        }
        min_us = base_us + ((min_us - base_us) * scale);
        max_us = base_us + ((max_us - base_us) * scale);
        motor_mix_scaled_count++;
    }

    if (min_us < floor_us)
    {
        shift_us = floor_us - min_us;
    }
    else if (max_us > ceiling_us)
    {
        shift_us = ceiling_us - max_us;
    }
    else
    {
        shift_us = 0.0f;
    }

    if (shift_us != 0.0f)
    {
        for (index = 0U; index < 4U; index++)
        {
            motor_us[index] += shift_us;
        }
        motor_mix_shifted_count++;
        if (MotorControl_Abs(shift_us) >
            MotorControl_Abs(motor_mix_shift_peak_us))
        {
            motor_mix_shift_peak_us = shift_us;
        }
    }
}

void MotorControl_Update(HAL_StatusTypeDef imu_status,
                         uint8_t imu_ready,
                         float roll_deg,
                         float pitch_deg,
                         float roll_pid_us,
                         float pitch_pid_us,
                         float yaw_pid_us)
{
    float base_us;
    float motor_mix_us[4];

    if (motor_arm_request != 1U)
    {
        MotorControl_ForceDisarm();
        return;
    }
    if (esc_pwm_started == 0U)
    {
        MotorControl_Block(MOTOR_CONTROL_BLOCKED_ESC);
        return;
    }
    if ((imu_ready == 0U) || (imu_status != HAL_OK))
    {
        MotorControl_Block(MOTOR_CONTROL_BLOCKED_IMU);
        return;
    }
    if ((MotorControl_IsFinite(roll_deg) == 0U) ||
        (MotorControl_IsFinite(pitch_deg) == 0U) ||
        (MotorControl_IsFinite(roll_pid_us) == 0U) ||
        (MotorControl_IsFinite(pitch_pid_us) == 0U) ||
        (MotorControl_IsFinite(yaw_pid_us) == 0U))
    {
        MotorControl_Block(MOTOR_CONTROL_BLOCKED_IMU);
        return;
    }

    motor_throttle_us = MotorControl_ClampThrottle(motor_throttle_us);
    if ((motor_armed == 0U) &&
        (motor_throttle_us > MOTOR_CONTROL_ARM_THROTTLE_MAX_US))
    {
        MotorControl_Block(MOTOR_CONTROL_BLOCKED_THROTTLE);
        return;
    }

    if ((motor_armed == 0U) &&
        ((MotorControl_Abs(roll_deg) > MOTOR_CONTROL_ARM_MAX_TILT_DEG) ||
         (MotorControl_Abs(pitch_deg) > MOTOR_CONTROL_ARM_MAX_TILT_DEG)))
    {
        MotorControl_Block(MOTOR_CONTROL_BLOCKED_TILT);
        return;
    }

    motor_roll_mix_us = MotorControl_ClampCorrection(roll_pid_us);
    motor_pitch_mix_us = MotorControl_ClampCorrection(pitch_pid_us);
    motor_yaw_mix_us = MotorControl_ClampCorrection(yaw_pid_us);

    if (motor_armed == 0U)
    {
        motor_roll_mix_peak_us = 0.0f;
        motor_pitch_mix_peak_us = 0.0f;
        motor_yaw_mix_peak_us = 0.0f;
        motor_roll_mix_peak_throttle_us = MOTOR_CONTROL_IDLE_US;
        motor_mix_shift_peak_us = 0.0f;
        motor_mix_shifted_count = 0U;
        motor_mix_scaled_count = 0U;
        motor_mix_peak_session_count++;
    }
    if (MotorControl_Abs(motor_roll_mix_us) >
        MotorControl_Abs(motor_roll_mix_peak_us))
    {
        motor_roll_mix_peak_us = motor_roll_mix_us;
        motor_roll_mix_peak_throttle_us = motor_throttle_us;
    }
    if (MotorControl_Abs(motor_pitch_mix_us) >
        MotorControl_Abs(motor_pitch_mix_peak_us))
    {
        motor_pitch_mix_peak_us = motor_pitch_mix_us;
    }
    if (MotorControl_Abs(motor_yaw_mix_us) >
        MotorControl_Abs(motor_yaw_mix_peak_us))
    {
        motor_yaw_mix_peak_us = motor_yaw_mix_us;
    }
    base_us = (float)MotorControl_ApplyThrottleRamp(motor_throttle_us);

    /* X mixer (top view):  M1 front-left CW   M2 front-right CCW
     *                      M3 rear-left CCW   M4 rear-right CW */
    motor_mix_us[0] = base_us + motor_roll_mix_us + motor_pitch_mix_us
                      - motor_yaw_mix_us;
    motor_mix_us[1] = base_us - motor_roll_mix_us + motor_pitch_mix_us
                      + motor_yaw_mix_us;
    motor_mix_us[2] = base_us + motor_roll_mix_us - motor_pitch_mix_us
                      + motor_yaw_mix_us;
    motor_mix_us[3] = base_us - motor_roll_mix_us - motor_pitch_mix_us
                      - motor_yaw_mix_us;

    MotorControl_NormalizeMix(motor_mix_us, base_us);

    ESC_PWM_SetAll(MotorControl_ClampOutput(motor_mix_us[0]),
                   MotorControl_ClampOutput(motor_mix_us[1]),
                   MotorControl_ClampOutput(motor_mix_us[2]),
                   MotorControl_ClampOutput(motor_mix_us[3]));
    motor_armed = 1U;
    motor_control_state = MOTOR_CONTROL_ARMED;
}
