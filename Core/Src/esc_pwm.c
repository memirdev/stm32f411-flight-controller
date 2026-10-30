
#include "esc_pwm.h"

#include <stddef.h>

volatile uint16_t esc_motor_us[ESC_PWM_CHANNEL_COUNT] =
{
    ESC_PWM_MIN_US, ESC_PWM_MIN_US, ESC_PWM_MIN_US, ESC_PWM_MIN_US
};
volatile uint8_t esc_pwm_started = 0U;
volatile uint32_t esc_pwm_config_corrected = 0U;

static TIM_HandleTypeDef *esc_timer = NULL;

static uint16_t ESC_PWM_Clamp(uint16_t pulse_us)
{
    if (pulse_us < ESC_PWM_MIN_US)
    {
        return ESC_PWM_MIN_US;
    }
    if (pulse_us > ESC_PWM_MAX_US)
    {
        return ESC_PWM_MAX_US;
    }
    return pulse_us;
}

uint32_t ESC_PWM_TimerClockHz(void)
{
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock_hz *= 2U;
    }

    return timer_clock_hz;
}

uint32_t ESC_PWM_TimerPrescaler(void)
{
    uint32_t timer_clock_hz = ESC_PWM_TimerClockHz();

    if (timer_clock_hz < ESC_PWM_TIMER_TICK_HZ)
    {
        return 0U;
    }

    return (timer_clock_hz / ESC_PWM_TIMER_TICK_HZ) - 1U;
}

HAL_StatusTypeDef ESC_PWM_Start(TIM_HandleTypeDef *htim)
{
    const uint32_t channels[ESC_PWM_CHANNEL_COUNT] =
    {
        TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
    };
    uint32_t index;

    uint32_t required_prescaler;

    if ((htim == NULL) || (htim->Instance != TIM3))
    {
        return HAL_ERROR;
    }

    required_prescaler = ESC_PWM_TimerPrescaler();
    if (((uint32_t)htim->Init.Prescaler != required_prescaler) ||
        (htim->Init.Period != ((uint32_t)ESC_PWM_PERIOD_US - 1U)))
    {
        esc_pwm_config_corrected++;
        htim->Init.Prescaler = required_prescaler;
        __HAL_TIM_SET_PRESCALER(htim, required_prescaler);
        __HAL_TIM_SET_AUTORELOAD(htim, (uint32_t)ESC_PWM_PERIOD_US - 1U);
        htim->Instance->EGR = TIM_EGR_UG;
    }

    if (((required_prescaler + 1U) * ESC_PWM_TIMER_TICK_HZ) !=
        ESC_PWM_TimerClockHz())
    {
        return HAL_ERROR;
    }

    esc_timer = htim;
    ESC_PWM_SetMinimum();

    for (index = 0U; index < ESC_PWM_CHANNEL_COUNT; index++)
    {
        if (HAL_TIM_PWM_Start(esc_timer, channels[index]) != HAL_OK)
        {
            while (index > 0U)
            {
                index--;
                (void)HAL_TIM_PWM_Stop(esc_timer, channels[index]);
            }
            esc_pwm_started = 0U;
            return HAL_ERROR;
        }
    }

    esc_pwm_started = 1U;
    return HAL_OK;
}

void ESC_PWM_SetAll(uint16_t motor1_us,
                    uint16_t motor2_us,
                    uint16_t motor3_us,
                    uint16_t motor4_us)
{
    if (esc_timer == NULL)
    {
        return;
    }

    esc_motor_us[0] = ESC_PWM_Clamp(motor1_us);
    esc_motor_us[1] = ESC_PWM_Clamp(motor2_us);
    esc_motor_us[2] = ESC_PWM_Clamp(motor3_us);
    esc_motor_us[3] = ESC_PWM_Clamp(motor4_us);

    __HAL_TIM_SET_COMPARE(esc_timer, TIM_CHANNEL_1, esc_motor_us[0]);
    __HAL_TIM_SET_COMPARE(esc_timer, TIM_CHANNEL_2, esc_motor_us[1]);
    __HAL_TIM_SET_COMPARE(esc_timer, TIM_CHANNEL_3, esc_motor_us[2]);
    __HAL_TIM_SET_COMPARE(esc_timer, TIM_CHANNEL_4, esc_motor_us[3]);
}

void ESC_PWM_SetMinimum(void)
{
    ESC_PWM_SetAll(ESC_PWM_MIN_US,
                   ESC_PWM_MIN_US,
                   ESC_PWM_MIN_US,
                   ESC_PWM_MIN_US);
}
