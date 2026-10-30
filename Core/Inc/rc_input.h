#ifndef RC_INPUT_H
#define RC_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RC_INPUT_ROLL_MAX_DEG       15.0f
#define RC_INPUT_PITCH_MAX_DEG      15.0f
#define RC_INPUT_YAW_MAX_DPS        60.0f

typedef enum
{
    RC_INPUT_STATUS_NOT_READY = 0U,
    RC_INPUT_STATUS_DISARMED,
    RC_INPUT_STATUS_ARM_SWITCH_ON,
    RC_INPUT_STATUS_SOURCE_LOST,
    RC_INPUT_STATUS_ARM_CHANNEL_INVALID,
    RC_INPUT_STATUS_CHANNEL_RANGE_ERROR,
    RC_INPUT_STATUS_SNAPSHOT_ERROR
} RC_InputStatus_t;

extern volatile RC_InputStatus_t rc_input_status;
extern volatile uint8_t rc_input_valid;
extern volatile uint8_t rc_input_arm_switch_on;
extern volatile uint8_t rc_input_throttle_low;
extern volatile uint8_t rc_input_sticks_centered;
extern volatile float rc_input_roll_normalized;
extern volatile float rc_input_pitch_normalized;
extern volatile float rc_input_yaw_normalized;
extern volatile float rc_input_throttle_normalized;
extern volatile float rc_input_roll_deg;
extern volatile float rc_input_pitch_deg;
extern volatile float rc_input_yaw_rate_dps;
extern volatile float rc_input_throttle_percent;
extern volatile uint32_t rc_input_update_count;
extern volatile uint32_t rc_input_last_frame_count;
extern volatile uint32_t rc_input_range_error_count;
extern volatile uint32_t rc_input_snapshot_error_count;
extern volatile uint32_t rc_input_snapshot_retry_count;

void RC_Input_Init(void);
void RC_Input_Update(void);

#ifdef __cplusplus
}
#endif

#endif
