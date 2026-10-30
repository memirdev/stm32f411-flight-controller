#include "rc_input.h"

#include "ibus_receiver.h"
#include "motor_control.h"

/* iBUS channels: CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5 arm */
#define RC_INPUT_ROLL_CHANNEL       0U
#define RC_INPUT_PITCH_CHANNEL      1U
#define RC_INPUT_THROTTLE_CHANNEL   2U
#define RC_INPUT_YAW_CHANNEL        3U
#define RC_INPUT_ARM_CHANNEL        4U
#define RC_INPUT_USED_CHANNEL_COUNT 5U
#define RC_INPUT_ROLL_MIN       1025U
#define RC_INPUT_ROLL_CENTER    1500U
#define RC_INPUT_ROLL_MAX       1997U
#define RC_INPUT_PITCH_MIN      1000U
#define RC_INPUT_PITCH_CENTER   1498U
#define RC_INPUT_PITCH_MAX      2000U
#define RC_INPUT_THROTTLE_MIN   1018U
#define RC_INPUT_THROTTLE_MAX   1968U
#define RC_INPUT_YAW_MIN        1000U
#define RC_INPUT_YAW_CENTER     1491U
#define RC_INPUT_YAW_MAX        1928U

#define RC_INPUT_CENTER_DEADBAND       15U
#define RC_INPUT_PLAUSIBLE_MIN         800U
#define RC_INPUT_PLAUSIBLE_MAX         2200U
#define RC_INPUT_ARM_DISARM_MAX        1250U
#define RC_INPUT_ARM_ON_MIN            1750U
#define RC_INPUT_ARM_STICK_MAX         0.03f
#define RC_INPUT_SNAPSHOT_MAX_ATTEMPTS 3U

volatile RC_InputStatus_t rc_input_status = RC_INPUT_STATUS_NOT_READY;
volatile uint8_t rc_input_valid = 0U;
volatile uint8_t rc_input_arm_switch_on = 0U;
volatile uint8_t rc_input_throttle_low = 0U;
volatile uint8_t rc_input_sticks_centered = 0U;
volatile float rc_input_roll_normalized = 0.0f;
volatile float rc_input_pitch_normalized = 0.0f;
volatile float rc_input_yaw_normalized = 0.0f;
volatile float rc_input_throttle_normalized = 0.0f;
volatile float rc_input_roll_deg = 0.0f;
volatile float rc_input_pitch_deg = 0.0f;
volatile float rc_input_yaw_rate_dps = 0.0f;
volatile float rc_input_throttle_percent = 0.0f;
volatile uint32_t rc_input_update_count = 0U;
volatile uint32_t rc_input_last_frame_count = 0U;
volatile uint32_t rc_input_range_error_count = 0U;
volatile uint32_t rc_input_snapshot_error_count = 0U;
volatile uint32_t rc_input_snapshot_retry_count = 0U;

static void RC_Input_SetSafe(RC_InputStatus_t status)
{
    rc_input_status = status;
    rc_input_valid = 0U;
    rc_input_arm_switch_on = 0U;
    rc_input_throttle_low = 1U;
    rc_input_sticks_centered = 0U;
    rc_input_roll_normalized = 0.0f;
    rc_input_pitch_normalized = 0.0f;
    rc_input_yaw_normalized = 0.0f;
    rc_input_throttle_normalized = 0.0f;
    rc_input_roll_deg = 0.0f;
    rc_input_pitch_deg = 0.0f;
    rc_input_yaw_rate_dps = 0.0f;
    rc_input_throttle_percent = 0.0f;
}

static float RC_Input_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float RC_Input_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float RC_Input_MapCentered(uint16_t raw,
                                  uint16_t minimum,
                                  uint16_t center,
                                  uint16_t maximum)
{
    uint16_t deadband_low = (uint16_t)(center - RC_INPUT_CENTER_DEADBAND);
    uint16_t deadband_high = (uint16_t)(center + RC_INPUT_CENTER_DEADBAND);
    float normalized;

    if ((raw >= deadband_low) && (raw <= deadband_high))
    {
        return 0.0f;
    }

    if (raw < deadband_low)
    {
        normalized = -((float)(deadband_low - raw) /
                       (float)(deadband_low - minimum));
    }
    else
    {
        normalized = (float)(raw - deadband_high) /
                     (float)(maximum - deadband_high);
    }

    return RC_Input_Clamp(normalized, -1.0f, 1.0f);
}

static float RC_Input_MapThrottle(uint16_t raw)
{
    float normalized = (float)((int32_t)raw - (int32_t)RC_INPUT_THROTTLE_MIN) /
                       (float)(RC_INPUT_THROTTLE_MAX - RC_INPUT_THROTTLE_MIN);
    return RC_Input_Clamp(normalized, 0.0f, 1.0f);
}

static uint8_t RC_Input_CopyChannels(uint16_t *snapshot)
{
    uint32_t frame_before;
    uint32_t frame_after;
    uint8_t attempt;
    uint8_t channel;

    for (attempt = 0U; attempt < RC_INPUT_SNAPSHOT_MAX_ATTEMPTS; attempt++)
    {
        frame_before = ibus_frame_count;
        __DMB();
        for (channel = 0U; channel < RC_INPUT_USED_CHANNEL_COUNT; channel++)
        {
            snapshot[channel] = ibus_channels[channel];
        }
        __DMB();
        frame_after = ibus_frame_count;

        if (frame_before == frame_after)
        {
            rc_input_last_frame_count = frame_after;
            return 1U;
        }
        rc_input_snapshot_retry_count++;
    }

    return 0U;
}

static uint8_t RC_Input_ChannelsPlausible(const uint16_t *channels)
{
    uint8_t channel;

    for (channel = 0U; channel < RC_INPUT_USED_CHANNEL_COUNT; channel++)
    {
        if ((channels[channel] < RC_INPUT_PLAUSIBLE_MIN) ||
            (channels[channel] > RC_INPUT_PLAUSIBLE_MAX))
        {
            return 0U;
        }
    }
    return 1U;
}

void RC_Input_Init(void)
{
    rc_input_update_count = 0U;
    rc_input_last_frame_count = 0U;
    rc_input_range_error_count = 0U;
    rc_input_snapshot_error_count = 0U;
    rc_input_snapshot_retry_count = 0U;
    RC_Input_SetSafe(RC_INPUT_STATUS_NOT_READY);
}

void RC_Input_Update(void)
{
    uint16_t channels[RC_INPUT_USED_CHANNEL_COUNT];
    uint16_t arm_raw;

    if ((ibus_link_valid == 0U) || (ibus_status != IBUS_STATUS_VALID))
    {
        RC_Input_SetSafe(RC_INPUT_STATUS_SOURCE_LOST);
        return;
    }

    if (RC_Input_CopyChannels(channels) == 0U)
    {
        rc_input_snapshot_error_count++;
        RC_Input_SetSafe(RC_INPUT_STATUS_SNAPSHOT_ERROR);
        return;
    }

    if (RC_Input_ChannelsPlausible(channels) == 0U)
    {
        rc_input_range_error_count++;
        RC_Input_SetSafe(RC_INPUT_STATUS_CHANNEL_RANGE_ERROR);
        return;
    }

    arm_raw = channels[RC_INPUT_ARM_CHANNEL];
    if ((arm_raw > RC_INPUT_ARM_DISARM_MAX) &&
        (arm_raw < RC_INPUT_ARM_ON_MIN))
    {
        RC_Input_SetSafe(RC_INPUT_STATUS_ARM_CHANNEL_INVALID);
        return;
    }

    rc_input_roll_normalized =
        RC_Input_MapCentered(channels[RC_INPUT_ROLL_CHANNEL],
                             RC_INPUT_ROLL_MIN,
                             RC_INPUT_ROLL_CENTER,
                             RC_INPUT_ROLL_MAX);
    /* Mode 2: right stick forward = nose down */
    rc_input_pitch_normalized =
        -RC_Input_MapCentered(channels[RC_INPUT_PITCH_CHANNEL],
                              RC_INPUT_PITCH_MIN,
                              RC_INPUT_PITCH_CENTER,
                              RC_INPUT_PITCH_MAX);
    rc_input_yaw_normalized =
        RC_Input_MapCentered(channels[RC_INPUT_YAW_CHANNEL],
                             RC_INPUT_YAW_MIN,
                             RC_INPUT_YAW_CENTER,
                             RC_INPUT_YAW_MAX);
    rc_input_throttle_normalized =
        RC_Input_MapThrottle(channels[RC_INPUT_THROTTLE_CHANNEL]);

    rc_input_roll_deg =
        rc_input_roll_normalized * RC_INPUT_ROLL_MAX_DEG;
    rc_input_pitch_deg =
        rc_input_pitch_normalized * RC_INPUT_PITCH_MAX_DEG;
    rc_input_yaw_rate_dps =
        rc_input_yaw_normalized * RC_INPUT_YAW_MAX_DPS;
    rc_input_throttle_percent = rc_input_throttle_normalized * 100.0f;

    rc_input_valid = 1U;
    rc_input_arm_switch_on = (arm_raw >= RC_INPUT_ARM_ON_MIN) ? 1U : 0U;
    rc_input_throttle_low =
        (MotorControl_ThrottleFromNormalized(rc_input_throttle_normalized) <=
         MOTOR_CONTROL_ARM_THROTTLE_MAX_US) ? 1U : 0U;
    rc_input_sticks_centered =
        ((RC_Input_Abs(rc_input_roll_normalized) <= RC_INPUT_ARM_STICK_MAX) &&
         (RC_Input_Abs(rc_input_pitch_normalized) <= RC_INPUT_ARM_STICK_MAX) &&
         (RC_Input_Abs(rc_input_yaw_normalized) <= RC_INPUT_ARM_STICK_MAX))
            ? 1U : 0U;
    rc_input_status = (rc_input_arm_switch_on != 0U)
                          ? RC_INPUT_STATUS_ARM_SWITCH_ON
                          : RC_INPUT_STATUS_DISARMED;
    rc_input_update_count++;
}
