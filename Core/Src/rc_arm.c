#include "rc_arm.h"

#include "rc_input.h"

volatile RC_ArmState_t rc_arm_state = RC_ARM_STATE_NOT_READY;
volatile uint8_t rc_arm_request = 0U;
volatile uint8_t rc_arm_rearm_required = 1U;
volatile uint8_t rc_arm_system_ready = 0U;
volatile uint32_t rc_arm_count = 0U;
volatile uint32_t rc_disarm_count = 0U;
volatile uint32_t rc_arm_throttle_block_count = 0U;
volatile uint32_t rc_arm_sticks_block_count = 0U;

static void RC_Arm_Disarm(uint8_t require_switch_off,
                          RC_ArmState_t state)
{
    if (rc_arm_request != 0U)
    {
        rc_disarm_count++;
    }

    rc_arm_request = 0U;
    rc_arm_rearm_required = require_switch_off;
    rc_arm_state = state;
}

void RC_Arm_Init(void)
{
    rc_arm_count = 0U;
    rc_disarm_count = 0U;
    rc_arm_throttle_block_count = 0U;
    rc_arm_sticks_block_count = 0U;
    rc_arm_request = 0U;
    rc_arm_rearm_required = 1U;
    rc_arm_system_ready = 0U;
    rc_arm_state = RC_ARM_STATE_NOT_READY;
}

void RC_Arm_Update(uint8_t system_ready)
{
    rc_arm_system_ready = (system_ready != 0U) ? 1U : 0U;

    if (rc_input_valid == 0U)
    {
        RC_Arm_Disarm(1U, RC_ARM_STATE_INPUT_LOST);
        return;
    }

    if (rc_input_arm_switch_on == 0U)
    {
        RC_Arm_Disarm(0U, RC_ARM_STATE_READY);
        return;
    }

    if (rc_arm_request != 0U)
    {
        if (rc_arm_system_ready == 0U)
        {
            RC_Arm_Disarm(1U, RC_ARM_STATE_SYSTEM_BLOCKED);
            return;
        }
        rc_arm_state = RC_ARM_STATE_ARMED;
        return;
    }

    if (rc_arm_rearm_required != 0U)
    {
        rc_arm_state = RC_ARM_STATE_WAIT_SWITCH_OFF;
        return;
    }

    if (rc_input_throttle_low == 0U)
    {
        rc_arm_throttle_block_count++;
        RC_Arm_Disarm(1U, RC_ARM_STATE_THROTTLE_BLOCKED);
        return;
    }

    if (rc_input_sticks_centered == 0U)
    {
        rc_arm_sticks_block_count++;
        RC_Arm_Disarm(1U, RC_ARM_STATE_STICKS_BLOCKED);
        return;
    }

    if (rc_arm_system_ready == 0U)
    {
        RC_Arm_Disarm(1U, RC_ARM_STATE_SYSTEM_BLOCKED);
        return;
    }

    rc_arm_request = 1U;
    rc_arm_rearm_required = 1U;
    rc_arm_state = RC_ARM_STATE_ARMED;
    rc_arm_count++;
}

void RC_Arm_ForceSystemBlock(void)
{
    RC_Arm_Disarm(1U, RC_ARM_STATE_SYSTEM_BLOCKED);
}
