#ifndef RC_ARM_H
#define RC_ARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    RC_ARM_STATE_NOT_READY = 0U,
    RC_ARM_STATE_WAIT_SWITCH_OFF,
    RC_ARM_STATE_READY,
    RC_ARM_STATE_ARMED,
    RC_ARM_STATE_THROTTLE_BLOCKED,
    RC_ARM_STATE_STICKS_BLOCKED,
    RC_ARM_STATE_INPUT_LOST,
    RC_ARM_STATE_SYSTEM_BLOCKED
} RC_ArmState_t;

extern volatile RC_ArmState_t rc_arm_state;
extern volatile uint8_t rc_arm_request;
extern volatile uint8_t rc_arm_rearm_required;
extern volatile uint8_t rc_arm_system_ready;
extern volatile uint32_t rc_arm_count;
extern volatile uint32_t rc_disarm_count;
extern volatile uint32_t rc_arm_throttle_block_count;
extern volatile uint32_t rc_arm_sticks_block_count;

/* Arming requires: switch on, low throttle, centered sticks, system ready.
 * After a disarm the switch must be turned off before arming again. */
void RC_Arm_Init(void);
void RC_Arm_Update(uint8_t system_ready);
void RC_Arm_ForceSystemBlock(void);

#ifdef __cplusplus
}
#endif

#endif
