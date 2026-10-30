#ifndef ESC_PWM_H
#define ESC_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
/* TIM3: M1 PA6, M2 PA7, M3 PB0, M4 PB1 (1 µs tick, 400 Hz) */
#define ESC_PWM_TIMER_TICK_HZ   1000000U
#define ESC_PWM_FREQUENCY_HZ    400U
#define ESC_PWM_PERIOD_US       (ESC_PWM_TIMER_TICK_HZ / ESC_PWM_FREQUENCY_HZ)
#define ESC_PWM_MIN_US          1000U
#define ESC_PWM_MAX_US          2000U
#define ESC_PWM_CHANNEL_COUNT 4U

#if ESC_PWM_MAX_US >= ESC_PWM_PERIOD_US
#error "ESC pulse must be shorter than the timer period"
#endif
#if (ESC_PWM_PERIOD_US * ESC_PWM_FREQUENCY_HZ) != ESC_PWM_TIMER_TICK_HZ
#error "ESC frame rate does not divide the 1 MHz timer tick"
#endif

extern volatile uint16_t esc_motor_us[ESC_PWM_CHANNEL_COUNT];
extern volatile uint8_t esc_pwm_started;
extern volatile uint32_t esc_pwm_config_corrected;

uint32_t ESC_PWM_TimerClockHz(void);

uint32_t ESC_PWM_TimerPrescaler(void);

HAL_StatusTypeDef ESC_PWM_Start(TIM_HandleTypeDef *htim);
void ESC_PWM_SetAll(uint16_t motor1_us,
                    uint16_t motor2_us,
                    uint16_t motor3_us,
                    uint16_t motor4_us);
void ESC_PWM_SetMinimum(void);

#ifdef __cplusplus
}
#endif

#endif
