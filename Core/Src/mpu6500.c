#include "mpu6500.h"

#define MPU6500_REG_SMPLRT_DIV       0x19U
#define MPU6500_REG_CONFIG           0x1AU
#define MPU6500_REG_GYRO_CONFIG      0x1BU
#define MPU6500_REG_ACCEL_CONFIG     0x1CU
#define MPU6500_REG_ACCEL_CONFIG2    0x1DU
#define MPU6500_REG_ACCEL_XOUT_H     0x3BU
#define MPU6500_REG_PWR_MGMT_1       0x6BU
#define MPU6500_REG_PWR_MGMT_2       0x6CU
#define MPU6500_REG_WHO_AM_I         0x75U

#define MPU6050_WHO_AM_I_VALUE       0x68U
#define MPU6500_WHO_AM_I_VALUE       0x70U
#define MPU6500_ACCEL_LSB_PER_G      16384.0f
#define MPU6500_GYRO_LSB_PER_DPS     16.4f

/* Settings: 1 kHz sampling, 41 Hz filter, gyro ±2000 °/s, accel ±2 g */
#define MPU6500_PWR_MGMT_1_H_RESET   0x80U
#define MPU6500_CFG_PWR_MGMT_1       0x01U
#define MPU6500_CFG_PWR_MGMT_2       0x00U
#define MPU6500_CFG_SMPLRT_DIV       0x00U
#define MPU6500_CFG_CONFIG           0x03U
#define MPU6500_CFG_GYRO_CONFIG      0x18U
#define MPU6500_CFG_ACCEL_CONFIG     0x00U
#define MPU6500_CFG_ACCEL_CONFIG2    0x03U

volatile MPU6500_Diagnostic_t mpu6500_diagnostic =
{
    0U, 0U, HAL_ERROR, HAL_ERROR, HAL_I2C_ERROR_NONE,
    0U, 0U, 0U, 0U, HAL_I2C_ERROR_NONE
};

#if MPU6500_USE_SPI
#include <string.h>

#define MPU6500_REG_SIGNAL_PATH_RESET 0x68U
#define MPU6500_REG_USER_CTRL         0x6AU
#define MPU6500_USER_CTRL_I2C_IF_DIS  0x10U
#define MPU6500_SIGNAL_PATH_RESET_ALL 0x07U
#define MPU6500_SPI_READ_BIT          0x80U

#define MPU6500_CS_GPIO_PORT          GPIOB
#define MPU6500_CS_PIN                GPIO_PIN_12
#define MPU6500_SPI_GPIO_PORT         GPIOB
#define MPU6500_SCK_PIN               GPIO_PIN_13
#define MPU6500_MISO_PIN              GPIO_PIN_14
#define MPU6500_MOSI_PIN              GPIO_PIN_15

#define MPU6500_SPI_MAX_HZ            900000U
#define MPU6500_SPI_TIMEOUT_MS        2U
#define MPU6500_SPI_MAX_READ          14U
#define MPU6500_CS_HOLD_LOOPS         32U
#define MPU6500_FROZEN_LIMIT          5U

typedef struct
{
    uint8_t reg;
    uint8_t expected;
} MPU6500_RegCheck_t;

static const MPU6500_RegCheck_t mpu6500_reg_checks[] =
{
    { MPU6500_REG_WHO_AM_I,      MPU6500_WHO_AM_I_VALUE },
    { MPU6500_REG_PWR_MGMT_1,    MPU6500_CFG_PWR_MGMT_1 },
    { MPU6500_REG_PWR_MGMT_2,    MPU6500_CFG_PWR_MGMT_2 },
    { MPU6500_REG_SMPLRT_DIV,    MPU6500_CFG_SMPLRT_DIV },
    { MPU6500_REG_CONFIG,        MPU6500_CFG_CONFIG },
    { MPU6500_REG_GYRO_CONFIG,   MPU6500_CFG_GYRO_CONFIG },
    { MPU6500_REG_ACCEL_CONFIG,  MPU6500_CFG_ACCEL_CONFIG },
    { MPU6500_REG_ACCEL_CONFIG2, MPU6500_CFG_ACCEL_CONFIG2 },
};
#define MPU6500_REG_CHECK_COUNT \
    ((uint8_t)(sizeof(mpu6500_reg_checks) / sizeof(mpu6500_reg_checks[0])))

volatile uint32_t mpu6500_spi_clock_hz = 0U;
volatile uint8_t mpu6500_spi_check_fail_reg = 0U;
volatile uint8_t mpu6500_spi_check_fail_value = 0U;
volatile uint32_t mpu6500_spi_stale_byte_count = 0U;
volatile uint8_t mpu6500_spi_last_stale_byte = 0U;

static uint32_t mpu6500_spi_error = MPU6500_SPI_ERR_NONE;
static uint8_t mpu6500_last_raw[MPU6500_SPI_MAX_READ];
static uint8_t mpu6500_frozen_count = 0U;
static uint8_t mpu6500_check_index = 0U;

static void MPU6500_SPI_CsHoldDelay(void)
{
    volatile uint32_t loop;

    for (loop = 0U; loop < MPU6500_CS_HOLD_LOOPS; loop++)
    {
        __NOP();
    }
}

static HAL_StatusTypeDef MPU6500_SPI_WaitFlag(uint32_t flag,
                                              FlagStatus wanted,
                                              uint32_t start_tick)
{
    while ((((SPI2->SR & flag) != 0U) ? SET : RESET) != wanted)
    {
        if ((HAL_GetTick() - start_tick) > MPU6500_SPI_TIMEOUT_MS)
        {
            mpu6500_spi_error = MPU6500_SPI_ERR_TIMEOUT;
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef MPU6500_SPI_Transfer(uint8_t *buffer, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t start_tick = HAL_GetTick();
    uint16_t index;

    mpu6500_spi_error = MPU6500_SPI_ERR_NONE;

    while ((SPI2->SR & SPI_SR_RXNE) != 0U)
    {
        mpu6500_spi_last_stale_byte = (uint8_t)SPI2->DR;
        mpu6500_spi_stale_byte_count++;
    }

    HAL_GPIO_WritePin(MPU6500_CS_GPIO_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET);
    for (index = 0U; (index < length) && (status == HAL_OK); index++)
    {
        status = MPU6500_SPI_WaitFlag(SPI_SR_TXE, SET, start_tick);
        if (status == HAL_OK)
        {
            SPI2->DR = buffer[index];
            status = MPU6500_SPI_WaitFlag(SPI_SR_RXNE, SET, start_tick);
        }
        if (status == HAL_OK)
        {
            buffer[index] = (uint8_t)SPI2->DR;
        }
    }
    if (status == HAL_OK)
    {
        status = MPU6500_SPI_WaitFlag(SPI_SR_TXE, SET, start_tick);
    }
    if (status == HAL_OK)
    {
        status = MPU6500_SPI_WaitFlag(SPI_SR_BSY, RESET, start_tick);
    }
    MPU6500_SPI_CsHoldDelay();
    HAL_GPIO_WritePin(MPU6500_CS_GPIO_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);
    return status;
}

static HAL_StatusTypeDef MPU6500_SPI_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t frame[2];

    frame[0] = (uint8_t)(reg & (uint8_t)~MPU6500_SPI_READ_BIT);
    frame[1] = value;
    return MPU6500_SPI_Transfer(frame, 2U);
}

static HAL_StatusTypeDef MPU6500_SPI_ReadRegisters(uint8_t start_reg,
                                                   uint8_t *buffer,
                                                   uint16_t length)
{
    uint8_t frame[MPU6500_SPI_MAX_READ + 1U];
    HAL_StatusTypeDef status;
    uint16_t index;

    if ((buffer == NULL) || (length == 0U) || (length > MPU6500_SPI_MAX_READ))
    {
        return HAL_ERROR;
    }

    frame[0] = (uint8_t)(start_reg | MPU6500_SPI_READ_BIT);
    for (index = 1U; index <= length; index++)
    {
        frame[index] = 0x00U;
    }
    status = MPU6500_SPI_Transfer(frame, (uint16_t)(length + 1U));
    if (status == HAL_OK)
    {
        for (index = 0U; index < length; index++)
        {
            buffer[index] = frame[index + 1U];
        }
    }
    return status;
}

static void MPU6500_SPI_Setup(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t pclk1_hz;
    uint32_t br = 0U;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(MPU6500_CS_GPIO_PORT, MPU6500_CS_PIN, GPIO_PIN_SET);
    gpio.Pin = MPU6500_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MPU6500_CS_GPIO_PORT, &gpio);

    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();

    gpio.Pin = MPU6500_SCK_PIN | MPU6500_MOSI_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(MPU6500_SPI_GPIO_PORT, &gpio);

    gpio.Pin = MPU6500_MISO_PIN;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MPU6500_SPI_GPIO_PORT, &gpio);

    pclk1_hz = HAL_RCC_GetPCLK1Freq();
    while ((br < 7U) && ((pclk1_hz >> (br + 1U)) > MPU6500_SPI_MAX_HZ))
    {
        br++;
    }
    mpu6500_spi_clock_hz = pclk1_hz >> (br + 1U);

    SPI2->CR2 = 0U;
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_CPOL | SPI_CR1_CPHA | (br << SPI_CR1_BR_Pos);
    SPI2->CR1 |= SPI_CR1_SPE;

    memset(mpu6500_last_raw, 0, sizeof(mpu6500_last_raw));
    mpu6500_frozen_count = 0U;
    mpu6500_check_index = 0U;
    mpu6500_spi_error = MPU6500_SPI_ERR_NONE;
}

static HAL_StatusTypeDef MPU6500_SPI_VerifyConfig(void)
{
    HAL_StatusTypeDef status;
    uint8_t value = 0U;
    uint8_t index;

    for (index = 0U; index < MPU6500_REG_CHECK_COUNT; index++)
    {
        status = MPU6500_SPI_ReadRegisters(mpu6500_reg_checks[index].reg,
                                           &value, 1U);
        if (status != HAL_OK)
        {
            return status;
        }
        if (value != mpu6500_reg_checks[index].expected)
        {
            mpu6500_spi_check_fail_reg = mpu6500_reg_checks[index].reg;
            mpu6500_spi_check_fail_value = value;
            mpu6500_spi_error = MPU6500_SPI_ERR_VERIFY;
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

static uint8_t MPU6500_SPI_BusStuck(const uint8_t *raw, uint16_t length)
{
    uint16_t index;

    if ((raw[0] != 0x00U) && (raw[0] != 0xFFU))
    {
        return 0U;
    }
    for (index = 1U; index < length; index++)
    {
        if (raw[index] != raw[0])
        {
            return 0U;
        }
    }
    return 1U;
}

/* SPI has no ACK, so every read is also checked: frozen data and a
 * read-back of one configuration register in turn. */
static HAL_StatusTypeDef MPU6500_SPI_ReadChecks(const uint8_t *raw)
{
    const MPU6500_RegCheck_t *check;
    HAL_StatusTypeDef status;
    uint8_t value = 0U;

    if (memcmp(raw, mpu6500_last_raw, sizeof(mpu6500_last_raw)) == 0)
    {
        if (mpu6500_frozen_count < MPU6500_FROZEN_LIMIT)
        {
            mpu6500_frozen_count++;
        }
    }
    else
    {
        mpu6500_frozen_count = 0U;
        memcpy(mpu6500_last_raw, raw, sizeof(mpu6500_last_raw));
    }
    if (mpu6500_frozen_count >= MPU6500_FROZEN_LIMIT)
    {
        mpu6500_spi_error = MPU6500_SPI_ERR_FROZEN;
        return HAL_ERROR;
    }

    check = &mpu6500_reg_checks[mpu6500_check_index];
    mpu6500_check_index = (uint8_t)((mpu6500_check_index + 1U) %
                                    MPU6500_REG_CHECK_COUNT);
    status = MPU6500_SPI_ReadRegisters(check->reg, &value, 1U);
    if (status != HAL_OK)
    {
        return status;
    }
    if (value != check->expected)
    {
        mpu6500_spi_check_fail_reg = check->reg;
        mpu6500_spi_check_fail_value = value;
        mpu6500_spi_error = MPU6500_SPI_ERR_REG_CHECK;
        return HAL_ERROR;
    }
    return HAL_OK;
}

#define MPU6500_WriteRegister(hi2c, reg, value) \
    MPU6500_SPI_WriteRegister((reg), (value))
#define MPU6500_ReadRegisters(hi2c, start_reg, buffer, length, timeout_ms) \
    MPU6500_SPI_ReadRegisters((start_reg), (buffer), (length))
#define MPU6500_BUS_ERROR(hi2c)      (mpu6500_spi_error)

#else

#define MPU6500_BUS_ERROR(hi2c)      ((hi2c)->ErrorCode)

static HAL_StatusTypeDef MPU6500_WriteRegister(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg,
                                                uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c,
                             MPU6500_I2C_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             MPU6500_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6500_ReadRegisters(I2C_HandleTypeDef *hi2c,
                                                uint8_t start_reg,
                                                uint8_t *buffer,
                                                uint16_t length,
                                                uint32_t timeout_ms)
{
    return HAL_I2C_Mem_Read(hi2c,
                            MPU6500_I2C_ADDRESS,
                            start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            length,
                            timeout_ms);
}

#endif

static int16_t MPU6500_CombineBytes(uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)(((uint16_t)high_byte << 8U) | (uint16_t)low_byte);
}

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i = 0U;

    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    mpu6500_diagnostic.init_step = 1U;
    mpu6500_diagnostic.who_am_i = 0U;
    mpu6500_diagnostic.init_status = HAL_BUSY;
    mpu6500_diagnostic.i2c_error = HAL_I2C_ERROR_NONE;

#if MPU6500_USE_SPI
    MPU6500_SPI_Setup();
    mpu6500_diagnostic.init_step = 20U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_USER_CTRL,
                                   MPU6500_USER_CTRL_I2C_IF_DIS);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 120U;
        return status;
    }
#endif

    status = MPU6500_ReadRegisters(hi2c,
                                   MPU6500_REG_WHO_AM_I,
                                   &who_am_i,
                                   1U,
                                   MPU6500_I2C_TIMEOUT_MS);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    mpu6500_diagnostic.who_am_i = who_am_i;
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 101U;
        return status;
    }
#if MPU6500_USE_SPI
    if (who_am_i != MPU6500_WHO_AM_I_VALUE)
#else
    if ((who_am_i != MPU6050_WHO_AM_I_VALUE) &&
        (who_am_i != MPU6500_WHO_AM_I_VALUE))
#endif
    {
        mpu6500_diagnostic.init_step = 102U;
        mpu6500_diagnostic.init_status = HAL_ERROR;
        return HAL_ERROR;
    }

    mpu6500_diagnostic.init_step = 2U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_PWR_MGMT_1,
                                   MPU6500_PWR_MGMT_1_H_RESET);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 103U;
        return status;
    }
    HAL_Delay(100U);

#if MPU6500_USE_SPI
    mpu6500_diagnostic.init_step = 21U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_USER_CTRL,
                                   MPU6500_USER_CTRL_I2C_IF_DIS);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 121U;
        return status;
    }

    mpu6500_diagnostic.init_step = 22U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_SIGNAL_PATH_RESET,
                                   MPU6500_SIGNAL_PATH_RESET_ALL);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 122U;
        return status;
    }
    HAL_Delay(100U);
#endif

    mpu6500_diagnostic.init_step = 3U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_PWR_MGMT_1,
                                   MPU6500_CFG_PWR_MGMT_1);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 104U;
        return status;
    }

    mpu6500_diagnostic.init_step = 4U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_PWR_MGMT_2,
                                   MPU6500_CFG_PWR_MGMT_2);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 105U;
        return status;
    }

    mpu6500_diagnostic.init_step = 5U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_SMPLRT_DIV,
                                   MPU6500_CFG_SMPLRT_DIV);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 106U;
        return status;
    }

    mpu6500_diagnostic.init_step = 6U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_CONFIG,
                                   MPU6500_CFG_CONFIG);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 107U;
        return status;
    }

    mpu6500_diagnostic.init_step = 7U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_GYRO_CONFIG,
                                   MPU6500_CFG_GYRO_CONFIG);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 108U;
        return status;
    }

    mpu6500_diagnostic.init_step = 8U;
    status = MPU6500_WriteRegister(hi2c, MPU6500_REG_ACCEL_CONFIG,
                                   MPU6500_CFG_ACCEL_CONFIG);
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 109U;
        return status;
    }

    if (who_am_i == MPU6500_WHO_AM_I_VALUE)
    {
        mpu6500_diagnostic.init_step = 9U;
        status = MPU6500_WriteRegister(hi2c, MPU6500_REG_ACCEL_CONFIG2,
                                       MPU6500_CFG_ACCEL_CONFIG2);
        mpu6500_diagnostic.init_status = status;
        mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
        if (status != HAL_OK)
        {
            mpu6500_diagnostic.init_step = 110U;
            return status;
        }
    }

#if MPU6500_USE_SPI
    mpu6500_diagnostic.init_step = 23U;
    status = MPU6500_SPI_VerifyConfig();
    mpu6500_diagnostic.init_status = status;
    mpu6500_diagnostic.i2c_error = MPU6500_BUS_ERROR(hi2c);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.init_step = 123U;
        return status;
    }
#endif

    mpu6500_diagnostic.init_step = 100U;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6500_ReadAll(I2C_HandleTypeDef *hi2c,
                                  MPU6500_Data_t *data)
{
    HAL_StatusTypeDef status;
    uint32_t i2c_error;
    uint8_t raw[14];
    uint8_t attempt;

    if ((hi2c == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    mpu6500_diagnostic.read_attempt_count = 0U;
    mpu6500_diagnostic.last_retry_error = HAL_I2C_ERROR_NONE;
    for (attempt = 1U; attempt <= MPU6500_READ_MAX_ATTEMPTS; attempt++)
    {
        status = MPU6500_ReadRegisters(hi2c,
                                       MPU6500_REG_ACCEL_XOUT_H,
                                       raw,
                                       (uint16_t)sizeof(raw),
                                       MPU6500_I2C_READ_TIMEOUT_MS);
#if MPU6500_USE_SPI
        if ((status == HAL_OK) &&
            (MPU6500_SPI_BusStuck(raw, (uint16_t)sizeof(raw)) != 0U))
        {
            mpu6500_spi_error = MPU6500_SPI_ERR_STUCK_BUS;
            status = HAL_ERROR;
        }
#endif
        i2c_error = MPU6500_BUS_ERROR(hi2c);
        mpu6500_diagnostic.read_attempt_count = attempt;
        mpu6500_diagnostic.read_status = status;
        mpu6500_diagnostic.i2c_error = i2c_error;
        if (status == HAL_OK)
        {
            if (attempt > 1U)
            {
                mpu6500_diagnostic.read_recovered_count++;
            }
            break;
        }

#if MPU6500_USE_SPI
        if ((i2c_error != MPU6500_SPI_ERR_STUCK_BUS) ||
            (attempt >= MPU6500_READ_MAX_ATTEMPTS))
#else
        if ((i2c_error != HAL_I2C_ERROR_AF) ||
            (attempt >= MPU6500_READ_MAX_ATTEMPTS))
#endif
        {
            mpu6500_diagnostic.read_failure_count++;
            return status;
        }

        mpu6500_diagnostic.last_retry_error = i2c_error;
        mpu6500_diagnostic.read_retry_count++;
    }

#if MPU6500_USE_SPI
    status = MPU6500_SPI_ReadChecks(raw);
    if (status != HAL_OK)
    {
        mpu6500_diagnostic.read_status = status;
        mpu6500_diagnostic.i2c_error = mpu6500_spi_error;
        mpu6500_diagnostic.read_failure_count++;
        return status;
    }
#endif

    data->accel_raw_x = MPU6500_CombineBytes(raw[0], raw[1]);
    data->accel_raw_y = MPU6500_CombineBytes(raw[2], raw[3]);
    data->accel_raw_z = MPU6500_CombineBytes(raw[4], raw[5]);
    data->temperature_raw = MPU6500_CombineBytes(raw[6], raw[7]);
    data->gyro_raw_x = MPU6500_CombineBytes(raw[8], raw[9]);
    data->gyro_raw_y = MPU6500_CombineBytes(raw[10], raw[11]);
    data->gyro_raw_z = MPU6500_CombineBytes(raw[12], raw[13]);

    data->accel_x_g = (float)data->accel_raw_x / MPU6500_ACCEL_LSB_PER_G;
    data->accel_y_g = (float)data->accel_raw_y / MPU6500_ACCEL_LSB_PER_G;
    data->accel_z_g = (float)data->accel_raw_z / MPU6500_ACCEL_LSB_PER_G;

    if (mpu6500_diagnostic.who_am_i == MPU6500_WHO_AM_I_VALUE)
    {
        data->temperature_c = ((float)data->temperature_raw / 333.87f) + 21.0f;
    }
    else
    {
        data->temperature_c = ((float)data->temperature_raw / 340.0f) + 36.53f;
    }

    data->gyro_x_dps = (float)data->gyro_raw_x / MPU6500_GYRO_LSB_PER_DPS;
    data->gyro_y_dps = (float)data->gyro_raw_y / MPU6500_GYRO_LSB_PER_DPS;
    data->gyro_z_dps = (float)data->gyro_raw_z / MPU6500_GYRO_LSB_PER_DPS;

    return HAL_OK;
}
