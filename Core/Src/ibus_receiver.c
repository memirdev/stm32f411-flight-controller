#include "ibus_receiver.h"

#define IBUS_FRAME_SIZE_BYTE        0x20U
#define IBUS_COMMAND_CHANNELS_BYTE  0x40U
#define IBUS_CHECKSUM_INITIAL       0xFFFFU
#define IBUS_PARTIAL_FRAME_GAP_MS   3U

volatile uint16_t ibus_channels[IBUS_CHANNEL_COUNT] = {0U};
volatile IBUS_Status_t ibus_status = IBUS_STATUS_NOT_STARTED;
volatile uint8_t ibus_link_valid = 0U;
volatile uint32_t ibus_frame_count = 0U;
volatile uint32_t ibus_byte_count = 0U;
volatile uint32_t ibus_checksum_error_count = 0U;
volatile uint32_t ibus_format_error_count = 0U;
volatile uint32_t ibus_uart_error_count = 0U;
volatile uint32_t ibus_restart_error_count = 0U;
volatile uint32_t ibus_last_frame_tick = 0U;
volatile uint32_t ibus_frame_age_ms = 0U;
volatile uint32_t ibus_last_uart_error = HAL_UART_ERROR_NONE;
volatile HAL_StatusTypeDef ibus_receive_status = HAL_ERROR;

static UART_HandleTypeDef *ibus_uart = NULL;
static uint8_t ibus_rx_byte = 0U;
static uint8_t ibus_frame[IBUS_FRAME_LENGTH] = {0U};
static uint8_t ibus_frame_index = 0U;
static uint32_t ibus_last_byte_tick = 0U;

static void IBUS_Receiver_ResetParser(void)
{
    ibus_frame_index = 0U;
}

static uint8_t IBUS_Receiver_CheckFrame(void)
{
    uint16_t checksum = IBUS_CHECKSUM_INITIAL;
    uint16_t received_checksum;
    uint8_t index;

    if ((ibus_frame[0] != IBUS_FRAME_SIZE_BYTE) ||
        (ibus_frame[1] != IBUS_COMMAND_CHANNELS_BYTE))
    {
        return 0U;
    }

    for (index = 0U; index < (IBUS_FRAME_LENGTH - 2U); index++)
    {
        checksum = (uint16_t)(checksum - ibus_frame[index]);
    }

    received_checksum = (uint16_t)ibus_frame[IBUS_FRAME_LENGTH - 2U];
    received_checksum |=
        (uint16_t)((uint16_t)ibus_frame[IBUS_FRAME_LENGTH - 1U] << 8U);

    return (checksum == received_checksum) ? 1U : 0U;
}

static void IBUS_Receiver_PublishFrame(uint32_t now_tick)
{
    uint8_t channel;

    for (channel = 0U; channel < IBUS_CHANNEL_COUNT; channel++)
    {
        uint8_t low_index = (uint8_t)(2U + (channel * 2U));
        uint16_t value = (uint16_t)ibus_frame[low_index];
        value |= (uint16_t)((uint16_t)ibus_frame[low_index + 1U] << 8U);
        ibus_channels[channel] = value;
    }

    ibus_last_frame_tick = now_tick;
    ibus_frame_age_ms = 0U;
    ibus_link_valid = 1U;
    ibus_status = IBUS_STATUS_VALID;
    ibus_frame_count++;
}

static void IBUS_Receiver_ProcessByte(uint8_t byte, uint32_t now_tick)
{
    if ((ibus_frame_index != 0U) &&
        ((now_tick - ibus_last_byte_tick) > IBUS_PARTIAL_FRAME_GAP_MS))
    {
        IBUS_Receiver_ResetParser();
    }
    ibus_last_byte_tick = now_tick;
    ibus_byte_count++;

    if (ibus_frame_index == 0U)
    {
        if (byte == IBUS_FRAME_SIZE_BYTE)
        {
            ibus_frame[0] = byte;
            ibus_frame_index = 1U;
        }
        return;
    }

    if (ibus_frame_index == 1U)
    {
        if (byte != IBUS_COMMAND_CHANNELS_BYTE)
        {
            ibus_format_error_count++;
            ibus_frame_index = (byte == IBUS_FRAME_SIZE_BYTE) ? 1U : 0U;
            if (ibus_frame_index == 1U)
            {
                ibus_frame[0] = byte;
            }
            return;
        }
    }

    ibus_frame[ibus_frame_index] = byte;
    ibus_frame_index++;

    if (ibus_frame_index >= IBUS_FRAME_LENGTH)
    {
        if (IBUS_Receiver_CheckFrame() != 0U)
        {
            IBUS_Receiver_PublishFrame(now_tick);
        }
        else
        {
            ibus_checksum_error_count++;
        }
        IBUS_Receiver_ResetParser();
    }
}

HAL_StatusTypeDef IBUS_Receiver_Start(UART_HandleTypeDef *huart)
{
    uint8_t channel;

    if ((huart == NULL) || (huart->Instance != USART2))
    {
        ibus_status = IBUS_STATUS_NOT_STARTED;
        ibus_receive_status = HAL_ERROR;
        return HAL_ERROR;
    }

    ibus_uart = huart;
    ibus_frame_count = 0U;
    ibus_byte_count = 0U;
    ibus_checksum_error_count = 0U;
    ibus_format_error_count = 0U;
    ibus_uart_error_count = 0U;
    ibus_restart_error_count = 0U;
    ibus_last_frame_tick = HAL_GetTick();
    ibus_frame_age_ms = 0U;
    ibus_last_uart_error = HAL_UART_ERROR_NONE;
    ibus_link_valid = 0U;
    ibus_status = IBUS_STATUS_WAITING;
    IBUS_Receiver_ResetParser();

    for (channel = 0U; channel < IBUS_CHANNEL_COUNT; channel++)
    {
        ibus_channels[channel] = 0U;
    }

    ibus_receive_status = HAL_UART_Receive_IT(ibus_uart, &ibus_rx_byte, 1U);
    if (ibus_receive_status != HAL_OK)
    {
        ibus_status = IBUS_STATUS_UART_ERROR;
    }
    return ibus_receive_status;
}

void IBUS_Receiver_Update(uint32_t now_tick)
{
    if (ibus_status == IBUS_STATUS_NOT_STARTED)
    {
        ibus_link_valid = 0U;
        return;
    }

    if (ibus_status == IBUS_STATUS_UART_ERROR)
    {
        ibus_link_valid = 0U;
        if ((ibus_uart != NULL) &&
            (ibus_uart->RxState == HAL_UART_STATE_READY))
        {
            ibus_receive_status =
                HAL_UART_Receive_IT(ibus_uart, &ibus_rx_byte, 1U);
            if (ibus_receive_status == HAL_OK)
            {
                ibus_status = IBUS_STATUS_WAITING;
            }
            else
            {
                ibus_restart_error_count++;
            }
        }
        return;
    }

    if (ibus_frame_count == 0U)
    {
        ibus_frame_age_ms = now_tick - ibus_last_frame_tick;
        ibus_link_valid = 0U;
        ibus_status = IBUS_STATUS_WAITING;
        return;
    }

    ibus_frame_age_ms = now_tick - ibus_last_frame_tick;
    if (ibus_frame_age_ms > IBUS_LINK_TIMEOUT_MS)
    {
        ibus_link_valid = 0U;
        ibus_status = IBUS_STATUS_TIMEOUT;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((ibus_uart == NULL) || (huart != ibus_uart))
    {
        return;
    }

    IBUS_Receiver_ProcessByte(ibus_rx_byte, HAL_GetTick());
    ibus_receive_status = HAL_UART_Receive_IT(ibus_uart, &ibus_rx_byte, 1U);
    if (ibus_receive_status != HAL_OK)
    {
        ibus_restart_error_count++;
        ibus_link_valid = 0U;
        ibus_status = IBUS_STATUS_UART_ERROR;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((ibus_uart == NULL) || (huart != ibus_uart))
    {
        return;
    }

    ibus_last_uart_error = HAL_UART_GetError(huart);
    ibus_uart_error_count++;
    ibus_link_valid = 0U;
    ibus_status = IBUS_STATUS_UART_ERROR;
    IBUS_Receiver_ResetParser();

    if (huart->RxState == HAL_UART_STATE_READY)
    {
        ibus_receive_status = HAL_UART_Receive_IT(ibus_uart, &ibus_rx_byte, 1U);
        if (ibus_receive_status == HAL_OK)
        {
            ibus_status = IBUS_STATUS_WAITING;
        }
        else
        {
            ibus_restart_error_count++;
        }
    }
}
