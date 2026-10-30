#ifndef MPU6500_H
#define MPU6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 1 = MPU6500, SPI2 (PB12-PB15)   0 = MPU6050/6500, I2C1 (PB6/PB7) */
#ifndef MPU6500_USE_SPI
#define MPU6500_USE_SPI              1
#endif

#define MPU6500_SPI_ERR_NONE         0x000U
#define MPU6500_SPI_ERR_TIMEOUT      0x101U
#define MPU6500_SPI_ERR_STUCK_BUS    0x102U
#define MPU6500_SPI_ERR_FROZEN       0x103U
#define MPU6500_SPI_ERR_REG_CHECK    0x104U
#define MPU6500_SPI_ERR_VERIFY       0x105U

#define MPU6500_I2C_ADDRESS          (0x68U << 1)
#define MPU6500_I2C_TIMEOUT_MS       100U
#define MPU6500_I2C_READ_TIMEOUT_MS  10U
#define MPU6500_READ_MAX_ATTEMPTS    3U

typedef struct
{
    int16_t accel_raw_x;
    int16_t accel_raw_y;
    int16_t accel_raw_z;
    int16_t temperature_raw;
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} MPU6500_Data_t;

typedef struct
{
    uint8_t init_step;
    uint8_t who_am_i;
    HAL_StatusTypeDef init_status;
    HAL_StatusTypeDef read_status;
    uint32_t i2c_error;
    uint8_t read_attempt_count;
    uint32_t read_retry_count;
    uint32_t read_recovered_count;
    uint32_t read_failure_count;
    uint32_t last_retry_error;
} MPU6500_Diagnostic_t;

extern volatile MPU6500_Diagnostic_t mpu6500_diagnostic;

#if MPU6500_USE_SPI
extern volatile uint32_t mpu6500_spi_clock_hz;
extern volatile uint8_t mpu6500_spi_check_fail_reg;
extern volatile uint8_t mpu6500_spi_check_fail_value;
extern volatile uint32_t mpu6500_spi_stale_byte_count;
extern volatile uint8_t mpu6500_spi_last_stale_byte;
#endif

HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MPU6500_ReadAll(I2C_HandleTypeDef *hi2c,
                                  MPU6500_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif
