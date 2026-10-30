#ifndef IBUS_RECEIVER_H
#define IBUS_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define IBUS_CHANNEL_COUNT       14U
#define IBUS_FRAME_LENGTH        32U
#define IBUS_LINK_TIMEOUT_MS     100U

typedef enum
{
    IBUS_STATUS_NOT_STARTED = 0,
    IBUS_STATUS_WAITING,
    IBUS_STATUS_VALID,
    IBUS_STATUS_TIMEOUT,
    IBUS_STATUS_UART_ERROR
} IBUS_Status_t;

extern volatile uint16_t ibus_channels[IBUS_CHANNEL_COUNT];
extern volatile IBUS_Status_t ibus_status;
extern volatile uint8_t ibus_link_valid;
extern volatile uint32_t ibus_frame_count;
extern volatile uint32_t ibus_byte_count;
extern volatile uint32_t ibus_checksum_error_count;
extern volatile uint32_t ibus_format_error_count;
extern volatile uint32_t ibus_uart_error_count;
extern volatile uint32_t ibus_restart_error_count;
extern volatile uint32_t ibus_last_frame_tick;
extern volatile uint32_t ibus_frame_age_ms;
extern volatile uint32_t ibus_last_uart_error;
extern volatile HAL_StatusTypeDef ibus_receive_status;

HAL_StatusTypeDef IBUS_Receiver_Start(UART_HandleTypeDef *huart);
void IBUS_Receiver_Update(uint32_t now_tick);

#ifdef __cplusplus
}
#endif

#endif
