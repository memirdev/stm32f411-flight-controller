/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Quadcopter flight controller (STM32F411CEU6 Black Pill).
 * The gyro is calibrated at startup, then a 100 Hz loop runs:
 * RC input -> IMU -> attitude filter -> PID -> motor mixer -> ESCs. */
#include <math.h>

#include "esc_pwm.h"
#include "flight_diagnostics.h"
#include "ibus_receiver.h"
#include "imu_filter.h"
#include "motor_control.h"
#include "mpu6500.h"
#include "pid_controller.h"
#include "rc_arm.h"
#include "rc_input.h"

#ifndef MPU6500_USE_SPI
#error "MPU6500_USE_SPI undefined: include mpu6500.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  SYSTEM_CLOCK_SOURCE_NONE = 0U,
  SYSTEM_CLOCK_SOURCE_HSE_PLL,
  SYSTEM_CLOCK_SOURCE_HSI,
  SYSTEM_CLOCK_SOURCE_PLL_PRESET
} SystemClockSource_t;

typedef enum
{
  FLIGHT_STOP_REASON_NONE = 0U,
  FLIGHT_STOP_REASON_USER_DISARM,
  FLIGHT_STOP_REASON_RC_INPUT_LOST,
  FLIGHT_STOP_REASON_SYSTEM_NOT_READY,
  FLIGHT_STOP_REASON_MPU_READ_ERROR,
  FLIGHT_STOP_REASON_LOOP_TIMING_ERROR,
  FLIGHT_STOP_REASON_MOTOR_CONTROL_BLOCKED,
  FLIGHT_STOP_REASON_UNKNOWN
} FlightStopReason_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RC_COMMAND_MOTOR_OUTPUT_ENABLED 1
#define CONTROL_LOOP_PERIOD_MS          10U  /* 100 Hz */
/* Level offset: the angle the sensor reads while the drone is level.
 * Re-measure it if the sensor or its mounting changes. */
#define IMU_LEVEL_ROLL_OFFSET_DEG       0.0f
#define IMU_LEVEL_PITCH_OFFSET_DEG      0.0f
#define IMU_CALIB_MAX_GYRO_SPREAD_DPS   3.0f
#define IMU_CALIB_MAX_ACCEL_SPREAD_G    0.05f
#define IMU_CALIB_MAX_ATTEMPTS          3U

#define IMU_READ_FAIL_TOLERANCE         3U  /* motors stop after this many consecutive IMU errors */

#define MPU_POWER_UP_DELAY_MS           100U

/* Watchdog (IWDG): the MCU resets if the loop stalls for ~1 s. */
#define WATCHDOG_PRESCALER_CODE         3U
#define WATCHDOG_RELOAD_VALUE           1000U
#define WATCHDOG_KEY_START              0xCCCCU
#define WATCHDOG_KEY_WRITE_ACCESS       0x5555U
#define WATCHDOG_KEY_RELOAD             0xAAAAU
#define WATCHDOG_SYNC_TIMEOUT_MS        48U

#define RESET_CAUSE_FLAG_LPWR           0x01U
#define RESET_CAUSE_FLAG_WWDG           0x02U
#define RESET_CAUSE_FLAG_IWDG           0x04U
#define RESET_CAUSE_FLAG_SFT            0x08U
#define RESET_CAUSE_FLAG_POR            0x10U
#define RESET_CAUSE_FLAG_PIN            0x20U
#define RESET_CAUSE_FLAG_BOR            0x40U
#define IMU_SIGN_CHECK_TRIGGER_DPS      20.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile SystemClockSource_t system_clock_source = SYSTEM_CLOCK_SOURCE_NONE;
volatile HAL_StatusTypeDef system_clock_hse_status = HAL_ERROR;
volatile uint32_t system_hclk_hz;
volatile uint32_t system_pclk1_hz;
volatile uint32_t system_timer_clock_hz;
volatile uint32_t system_timer_prescaler;
MPU6500_Data_t mpu_data;
HAL_StatusTypeDef mpu_status;
IMU_Filter_t imu_filter;
HAL_StatusTypeDef imu_calibration_status;
uint8_t imu_filter_ready;
uint32_t imu_last_tick;
uint32_t control_loop_next_tick;
volatile uint32_t control_loop_schedule_late_count;
volatile uint32_t control_loop_schedule_resync_count;
volatile uint32_t control_loop_last_lateness_ms;
volatile float imu_control_roll_deg;
volatile float imu_control_pitch_deg;
volatile float imu_calib_bias_x_dps;
volatile float imu_calib_bias_y_dps;
volatile float imu_calib_bias_z_dps;
volatile float imu_calib_spread_x_dps;
volatile float imu_calib_spread_y_dps;
volatile float imu_calib_spread_z_dps;
volatile float imu_calib_accel_spread_g;
volatile uint32_t imu_calib_attempt_count;
volatile uint32_t imu_calib_reject_count;
PID_Controller_t roll_pid;
PID_Controller_t pitch_pid;
PID_Controller_t yaw_rate_pid;
volatile float roll_setpoint_deg;
volatile float pitch_setpoint_deg;
volatile float yaw_rate_setpoint_dps;
volatile float roll_pid_output;
volatile float pitch_pid_output;
volatile float yaw_pid_output;
HAL_StatusTypeDef esc_status;
HAL_StatusTypeDef ibus_start_status;
volatile uint8_t rc_command_preview_valid;
volatile uint8_t rc_command_motor_lock_active;
volatile uint8_t rc_command_system_ready;
volatile uint16_t rc_command_throttle_preview_us;
#define FLIGHT_YAW_SATURATION_EPSILON_US 0.5f
volatile uint32_t flight_yaw_armed_loop_count;
volatile uint32_t flight_yaw_saturated_count;
volatile float flight_yaw_integral_peak;
volatile float flight_yaw_rate_mean_dps;
static uint8_t flight_yaw_session_active;
#define FLIGHT_YAW_CENTERED_SETPOINT_DPS 0.5f
volatile float flight_yaw_output_mean_us;
volatile float flight_roll_integral_peak;
volatile float flight_pitch_integral_peak;
#define PID_INTEGRAL_ENABLE_THROTTLE_US 1250U
/* ESC calibration mode: within 15 s after power-up, with arm off, holding
 * throttle above 95% for 2 s passes throttle straight to the ESCs. NO PROPS! */
#define ESC_CALIBRATION_WINDOW_MS    15000U
#define ESC_CALIBRATION_HOLD_MS       2000U
#define ESC_CALIBRATION_THROTTLE_MIN   0.95f
volatile uint8_t esc_calibration_active;
volatile uint16_t esc_calibration_pulse_us;
volatile uint32_t esc_calibration_pass_count;
static uint8_t esc_calibration_hold_active;
static uint32_t esc_calibration_hold_start_tick;
volatile FlightStopReason_t flight_stop_reason;
volatile uint32_t flight_stop_count;
volatile uint8_t flight_stop_session_active;
volatile RC_ArmState_t flight_stop_rc_arm_state;
volatile RC_InputStatus_t flight_stop_rc_input_status;
volatile IBUS_Status_t flight_stop_ibus_status;
volatile MotorControl_State_t flight_stop_motor_control_state;
volatile HAL_StatusTypeDef flight_stop_mpu_status;
volatile HAL_StatusTypeDef flight_stop_mpu_read_status;
volatile uint8_t flight_stop_rc_input_valid;
volatile uint8_t flight_stop_ibus_link_valid;
volatile uint8_t flight_stop_system_ready;
volatile uint32_t flight_stop_i2c_error;
volatile uint32_t flight_stop_hi2c_error;
volatile uint32_t flight_stop_ibus_frame_age_ms;
volatile float flight_stop_roll_deg;
volatile float flight_stop_pitch_deg;
volatile float flight_stop_throttle_percent;
volatile uint16_t flight_stop_motor_throttle_us;
volatile uint16_t flight_stop_esc_motor_us[ESC_PWM_CHANNEL_COUNT];

volatile uint8_t imu_read_fail_streak;
volatile uint32_t imu_read_coast_count;

volatile uint8_t watchdog_started;
volatile uint32_t main_loop_pass_count;
volatile uint32_t imu_recovery_pass_count;
volatile uint32_t imu_init_start_tick;
volatile uint32_t imu_init_extra_delay_ms;

/* Axis sign test. Expected: roll + right side down, pitch + nose up,
 * yaw + clockwise seen from above. */
volatile float imu_sign_check_gyro_min_dps[3];
volatile float imu_sign_check_gyro_max_dps[3];
volatile float imu_sign_check_first_dps[3];
volatile uint8_t imu_sign_check_first_captured[3];

volatile uint8_t reset_cause_flags;
volatile uint8_t reset_cause_was_watchdog;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef SystemClock_ConfigHsePll(void);
static void SystemClock_UpgradeToHsePll(void);
#if !MPU6500_USE_SPI
static HAL_StatusTypeDef I2C1_BusRecover(void);
#endif
static HAL_StatusTypeDef IMU_CalibrateGyroOnce(uint16_t sample_count);
static HAL_StatusTypeDef IMU_CalibrateGyro(uint16_t sample_count);
static uint8_t RC_CommandSystemReady(void);
static void RC_CommandForceSystemDisarm(void);
static void ControlLoop_WaitNext(void);
static void FlightStopDiagnostics_Init(void);
static void FlightStopDiagnostics_Latch(FlightStopReason_t reason);
static FlightStopReason_t FlightStopDiagnostics_RCReason(void);

static void ResetCause_Capture(void);
static void Watchdog_Start(void);
static void Watchdog_Refresh(void);
static void IMU_SignCheck_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef SystemClock_ConfigHsePll(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return HAL_ERROR;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  return HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

/* Switch to 96 MHz (HSE + PLL); stays at 16 MHz if the HSE fails. */
static void SystemClock_UpgradeToHsePll(void)
{
  if (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK)
  {
    system_clock_hse_status = HAL_OK;
    system_clock_source = SYSTEM_CLOCK_SOURCE_PLL_PRESET;
  }
  else
  {
    system_clock_hse_status = SystemClock_ConfigHsePll();
    system_clock_source = (system_clock_hse_status == HAL_OK)
                            ? SYSTEM_CLOCK_SOURCE_HSE_PLL
                            : SYSTEM_CLOCK_SOURCE_HSI;
  }

  system_hclk_hz = HAL_RCC_GetHCLKFreq();
  system_pclk1_hz = HAL_RCC_GetPCLK1Freq();
  system_timer_clock_hz = ESC_PWM_TimerClockHz();
  system_timer_prescaler = ESC_PWM_TimerPrescaler();
}

#if !MPU6500_USE_SPI
static HAL_StatusTypeDef I2C1_BusRecover(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t pulse;

  (void)HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(1);
  __HAL_RCC_I2C1_RELEASE_RESET();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_Delay(1);

  for (pulse = 0U; pulse < 9U; pulse++)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
    {
      break;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1);

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  return HAL_I2C_Init(&hi2c1);
}
#endif

static HAL_StatusTypeDef IMU_CalibrateGyroOnce(uint16_t sample_count)
{
  MPU6500_Data_t sample;
  HAL_StatusTypeDef status;
  float gyro_sum_x = 0.0f;
  float gyro_sum_y = 0.0f;
  float gyro_sum_z = 0.0f;
  float gyro_min_x = 0.0f;
  float gyro_max_x = 0.0f;
  float gyro_min_y = 0.0f;
  float gyro_max_y = 0.0f;
  float gyro_min_z = 0.0f;
  float gyro_max_z = 0.0f;
  float accel_min_g = 0.0f;
  float accel_max_g = 0.0f;
  uint16_t index;

  if (sample_count == 0U)
  {
    return HAL_ERROR;
  }

  for (index = 0U; index < sample_count; index++)
  {
    float accel_norm_g;

    status = MPU6500_ReadAll(&hi2c1, &sample);
    if (status != HAL_OK)
    {
      return status;
    }

    gyro_sum_x += sample.gyro_x_dps;
    gyro_sum_y += sample.gyro_y_dps;
    gyro_sum_z += sample.gyro_z_dps;

    accel_norm_g = sqrtf((sample.accel_x_g * sample.accel_x_g) +
                         (sample.accel_y_g * sample.accel_y_g) +
                         (sample.accel_z_g * sample.accel_z_g));

    if (index == 0U)
    {
      gyro_min_x = sample.gyro_x_dps;
      gyro_max_x = sample.gyro_x_dps;
      gyro_min_y = sample.gyro_y_dps;
      gyro_max_y = sample.gyro_y_dps;
      gyro_min_z = sample.gyro_z_dps;
      gyro_max_z = sample.gyro_z_dps;
      accel_min_g = accel_norm_g;
      accel_max_g = accel_norm_g;
    }
    else
    {
      if (sample.gyro_x_dps < gyro_min_x) { gyro_min_x = sample.gyro_x_dps; }
      if (sample.gyro_x_dps > gyro_max_x) { gyro_max_x = sample.gyro_x_dps; }
      if (sample.gyro_y_dps < gyro_min_y) { gyro_min_y = sample.gyro_y_dps; }
      if (sample.gyro_y_dps > gyro_max_y) { gyro_max_y = sample.gyro_y_dps; }
      if (sample.gyro_z_dps < gyro_min_z) { gyro_min_z = sample.gyro_z_dps; }
      if (sample.gyro_z_dps > gyro_max_z) { gyro_max_z = sample.gyro_z_dps; }
      if (accel_norm_g < accel_min_g) { accel_min_g = accel_norm_g; }
      if (accel_norm_g > accel_max_g) { accel_max_g = accel_norm_g; }
    }

    HAL_Delay(8);
  }

  imu_calib_bias_x_dps = gyro_sum_x / (float)sample_count;
  imu_calib_bias_y_dps = gyro_sum_y / (float)sample_count;
  imu_calib_bias_z_dps = gyro_sum_z / (float)sample_count;
  imu_calib_spread_x_dps = gyro_max_x - gyro_min_x;
  imu_calib_spread_y_dps = gyro_max_y - gyro_min_y;
  imu_calib_spread_z_dps = gyro_max_z - gyro_min_z;
  imu_calib_accel_spread_g = accel_max_g - accel_min_g;

  if ((imu_calib_spread_x_dps > IMU_CALIB_MAX_GYRO_SPREAD_DPS) ||
      (imu_calib_spread_y_dps > IMU_CALIB_MAX_GYRO_SPREAD_DPS) ||
      (imu_calib_spread_z_dps > IMU_CALIB_MAX_GYRO_SPREAD_DPS) ||
      (imu_calib_accel_spread_g > IMU_CALIB_MAX_ACCEL_SPREAD_G))
  {
    return HAL_BUSY;
  }

  IMU_Filter_SetGyroBias(&imu_filter,
                         imu_calib_bias_x_dps,
                         imu_calib_bias_y_dps,
                         imu_calib_bias_z_dps);

  return HAL_OK;
}

static HAL_StatusTypeDef IMU_CalibrateGyro(uint16_t sample_count)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t attempt;

  for (attempt = 0U; attempt < IMU_CALIB_MAX_ATTEMPTS; attempt++)
  {
    imu_calib_attempt_count++;
    status = IMU_CalibrateGyroOnce(sample_count);
    if (status != HAL_BUSY)
    {
      return status;
    }

    imu_calib_reject_count++;
    HAL_Delay(500);
  }

  return HAL_BUSY;
}

static uint8_t RC_CommandSystemReady(void)
{
  float roll_deg = imu_control_roll_deg;
  float pitch_deg = imu_control_pitch_deg;

  if ((esc_status != HAL_OK) || (esc_pwm_started == 0U) ||
      (mpu_status != HAL_OK) || (imu_filter_ready == 0U) ||
      (imu_filter.initialized == 0U) ||
      (roll_deg != roll_deg) || (pitch_deg != pitch_deg))
  {
    return 0U;
  }

  if ((rc_arm_request == 0U) &&
      ((roll_deg > MOTOR_CONTROL_ARM_MAX_TILT_DEG) ||
       (roll_deg < -MOTOR_CONTROL_ARM_MAX_TILT_DEG) ||
       (pitch_deg > MOTOR_CONTROL_ARM_MAX_TILT_DEG) ||
       (pitch_deg < -MOTOR_CONTROL_ARM_MAX_TILT_DEG)))
  {
    return 0U;
  }

  return 1U;
}

static void RC_CommandForceSystemDisarm(void)
{
  MotorControl_ForceDisarm();
  if (rc_arm_request != 0U)
  {
    RC_Arm_ForceSystemBlock();
  }
}

static void FlightStopDiagnostics_Init(void)
{
  uint32_t index;

  flight_stop_reason = FLIGHT_STOP_REASON_NONE;
  flight_stop_count = 0U;
  flight_stop_session_active = 0U;
  flight_stop_rc_arm_state = RC_ARM_STATE_NOT_READY;
  flight_stop_rc_input_status = RC_INPUT_STATUS_NOT_READY;
  flight_stop_ibus_status = IBUS_STATUS_NOT_STARTED;
  flight_stop_motor_control_state = MOTOR_CONTROL_DISARMED;
  flight_stop_mpu_status = HAL_ERROR;
  flight_stop_mpu_read_status = HAL_ERROR;
  flight_stop_rc_input_valid = 0U;
  flight_stop_ibus_link_valid = 0U;
  flight_stop_system_ready = 0U;
  flight_stop_i2c_error = 0U;
  flight_stop_hi2c_error = 0U;
  flight_stop_ibus_frame_age_ms = 0U;
  flight_stop_roll_deg = 0.0f;
  flight_stop_pitch_deg = 0.0f;
  flight_stop_throttle_percent = 0.0f;
  flight_stop_motor_throttle_us = MOTOR_CONTROL_IDLE_US;
  for (index = 0U; index < ESC_PWM_CHANNEL_COUNT; index++)
  {
    flight_stop_esc_motor_us[index] = ESC_PWM_MIN_US;
  }
}

static void FlightStopDiagnostics_Latch(FlightStopReason_t reason)
{
  uint32_t index;

  if ((flight_stop_session_active == 0U) ||
      (reason == FLIGHT_STOP_REASON_NONE))
  {
    return;
  }

  flight_stop_reason = reason;
  flight_stop_count++;
  flight_stop_rc_arm_state = rc_arm_state;
  flight_stop_rc_input_status = rc_input_status;
  flight_stop_ibus_status = ibus_status;
  flight_stop_motor_control_state = motor_control_state;
  flight_stop_mpu_status = mpu_status;
  flight_stop_mpu_read_status = mpu6500_diagnostic.read_status;
  flight_stop_rc_input_valid = rc_input_valid;
  flight_stop_ibus_link_valid = ibus_link_valid;
  flight_stop_system_ready = rc_command_system_ready;
  flight_stop_i2c_error = mpu6500_diagnostic.i2c_error;
  flight_stop_hi2c_error = hi2c1.ErrorCode;
  flight_stop_ibus_frame_age_ms = ibus_frame_age_ms;
  flight_stop_roll_deg = imu_control_roll_deg;
  flight_stop_pitch_deg = imu_control_pitch_deg;
  flight_stop_throttle_percent = rc_input_throttle_percent;
  flight_stop_motor_throttle_us = motor_throttle_us;
  for (index = 0U; index < ESC_PWM_CHANNEL_COUNT; index++)
  {
    flight_stop_esc_motor_us[index] = esc_motor_us[index];
  }
  flight_stop_session_active = 0U;
}

static FlightStopReason_t FlightStopDiagnostics_RCReason(void)
{
  if ((rc_input_valid == 0U) ||
      (rc_arm_state == RC_ARM_STATE_INPUT_LOST))
  {
    return FLIGHT_STOP_REASON_RC_INPUT_LOST;
  }
  if ((rc_command_system_ready == 0U) ||
      (rc_arm_state == RC_ARM_STATE_SYSTEM_BLOCKED))
  {
    return FLIGHT_STOP_REASON_SYSTEM_NOT_READY;
  }
  if (rc_input_arm_switch_on == 0U)
  {
    return FLIGHT_STOP_REASON_USER_DISARM;
  }
  return FLIGHT_STOP_REASON_UNKNOWN;
}

static void ControlLoop_WaitNext(void)
{
  uint32_t now_tick = HAL_GetTick();
  int32_t wait_ms = (int32_t)(control_loop_next_tick - now_tick);

  if (wait_ms > 0)
  {
    HAL_Delay((uint32_t)wait_ms);
    control_loop_last_lateness_ms = 0U;
  }
  else
  {
    uint32_t lateness_ms = now_tick - control_loop_next_tick;

    control_loop_last_lateness_ms = lateness_ms;
    if (lateness_ms > 0U)
    {
      control_loop_schedule_late_count++;
    }
    if (lateness_ms >= CONTROL_LOOP_PERIOD_MS)
    {
      control_loop_next_tick = now_tick;
      control_loop_schedule_resync_count++;
    }
  }

  control_loop_next_tick += CONTROL_LOOP_PERIOD_MS;
}

static void ResetCause_Capture(void)
{
  uint8_t flags = 0U;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET)
  {
    flags |= RESET_CAUSE_FLAG_LPWR;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    flags |= RESET_CAUSE_FLAG_WWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    flags |= RESET_CAUSE_FLAG_IWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    flags |= RESET_CAUSE_FLAG_SFT;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
  {
    flags |= RESET_CAUSE_FLAG_POR;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
  {
    flags |= RESET_CAUSE_FLAG_PIN;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
  {
    flags |= RESET_CAUSE_FLAG_BOR;
  }

  reset_cause_flags = flags;
  reset_cause_was_watchdog =
      ((flags & RESET_CAUSE_FLAG_IWDG) != 0U) ? 1U : 0U;

  __HAL_RCC_CLEAR_RESET_FLAGS();
}

static void Watchdog_Start(void)
{
  uint32_t start_tick;

  if (watchdog_started != 0U)
  {
    return;
  }

  IWDG->KR = WATCHDOG_KEY_START;
  IWDG->KR = WATCHDOG_KEY_WRITE_ACCESS;
  IWDG->PR = WATCHDOG_PRESCALER_CODE;
  IWDG->RLR = WATCHDOG_RELOAD_VALUE;

  start_tick = HAL_GetTick();
  while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U)
  {
    if ((HAL_GetTick() - start_tick) > WATCHDOG_SYNC_TIMEOUT_MS)
    {
      break;
    }
  }

  IWDG->KR = WATCHDOG_KEY_RELOAD;
  watchdog_started = 1U;
}

static void Watchdog_Refresh(void)
{
  if (watchdog_started == 0U)
  {
    return;
  }

  IWDG->KR = WATCHDOG_KEY_RELOAD;
}

static void IMU_SignCheck_Update(void)
{
  float axis[3];
  uint8_t index;

  axis[0] = imu_filter.gyro_x_dps;
  axis[1] = imu_filter.gyro_y_dps;
  axis[2] = imu_filter.gyro_z_dps;

  for (index = 0U; index < 3U; index++)
  {
    if (axis[index] < imu_sign_check_gyro_min_dps[index])
    {
      imu_sign_check_gyro_min_dps[index] = axis[index];
    }
    if (axis[index] > imu_sign_check_gyro_max_dps[index])
    {
      imu_sign_check_gyro_max_dps[index] = axis[index];
    }
    if ((imu_sign_check_first_captured[index] == 0U) &&
        ((axis[index] > IMU_SIGN_CHECK_TRIGGER_DPS) ||
         (axis[index] < -IMU_SIGN_CHECK_TRIGGER_DPS)))
    {
      imu_sign_check_first_dps[index] = axis[index];
      imu_sign_check_first_captured[index] = 1U;
    }
  }
}

static void EscCalibration_Update(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint16_t pulse_us;
  float normalized;

  if (esc_calibration_active != 0U)
  {
    if (rc_input_valid != 0U)
    {
      normalized = rc_input_throttle_normalized;
      if (normalized < 0.0f)
      {
        normalized = 0.0f;
      }
      if (normalized > 1.0f)
      {
        normalized = 1.0f;
      }
      pulse_us = (uint16_t)((float)ESC_PWM_MIN_US +
                            (normalized *
                             (float)(ESC_PWM_MAX_US - ESC_PWM_MIN_US)));
    }
    else
    {
      pulse_us = (uint16_t)ESC_PWM_MIN_US;
    }
    esc_calibration_pulse_us = pulse_us;
    ESC_PWM_SetAll(pulse_us, pulse_us, pulse_us, pulse_us);
    esc_calibration_pass_count++;
    HAL_Delay(5);
    return;
  }

  if (now_tick > (uint32_t)ESC_CALIBRATION_WINDOW_MS)
  {
    esc_calibration_hold_active = 0U;
    return;
  }

  if ((rc_input_valid != 0U) &&
      (rc_input_arm_switch_on == 0U) &&
      (rc_input_throttle_normalized >= ESC_CALIBRATION_THROTTLE_MIN))
  {
    if (esc_calibration_hold_active == 0U)
    {
      esc_calibration_hold_active = 1U;
      esc_calibration_hold_start_tick = now_tick;
    }
    else if ((now_tick - esc_calibration_hold_start_tick) >=
             (uint32_t)ESC_CALIBRATION_HOLD_MS)
    {
      esc_calibration_active = 1U;
    }
    else
    {
    }
  }
  else
  {
    esc_calibration_hold_active = 0U;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemClock_UpgradeToHsePll();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  ResetCause_Capture();
  __HAL_DBGMCU_FREEZE_IWDG();
  ibus_start_status = IBUS_Receiver_Start(&huart2);
  esc_status = ESC_PWM_Start(&htim3);
  MotorControl_Init();
  RC_Input_Init();
  RC_Arm_Init();
  IMU_Filter_Init(&imu_filter,
                  20.0f,   /* accel low-pass (Hz) */
                  60.0f,   /* gyro low-pass (Hz) */
                  1.5f);   /* complementary filter tau (s) */
  FlightDiagnostics_Init();
  FlightStopDiagnostics_Init();
  /* Roll: angle control */
  PID_Controller_Init(&roll_pid,
                      1.5f,    /* Kp */
                      0.6f,    /* Ki */
                      0.60f,   /* Kd */
                      50.0f,   /* integral limit */
                      (float)MOTOR_CONTROL_CORRECTION_LIMIT_US);  /* output limit (µs) */
  /* Pitch: angle control */
  PID_Controller_Init(&pitch_pid,
                      1.5f,    /* Kp */
                      0.6f,    /* Ki */
                      0.60f,   /* Kd */
                      50.0f,   /* integral limit */
                      (float)MOTOR_CONTROL_CORRECTION_LIMIT_US);  /* output limit (µs) */
  /* Yaw: rate control */
  PID_Controller_Init(&yaw_rate_pid,
                      2.0f,    /* Kp */
                      1.0f,    /* Ki */
                      0.0f,    /* Kd */
                      40.0f,   /* integral limit */
                      100.0f); /* output limit (µs) */
  roll_setpoint_deg = 0.0f;
  pitch_setpoint_deg = 0.0f;
  yaw_rate_setpoint_dps = 0.0f;
  roll_pid_output = 0.0f;
  pitch_pid_output = 0.0f;
  yaw_pid_output = 0.0f;
  rc_command_preview_valid = 0U;
  rc_command_motor_lock_active =
      (RC_COMMAND_MOTOR_OUTPUT_ENABLED != 0) ? 0U : 1U;
  rc_command_system_ready = 0U;
  rc_command_throttle_preview_us = MOTOR_CONTROL_IDLE_US;
  imu_calibration_status = HAL_BUSY;
  imu_filter_ready = 0U;
  imu_last_tick = HAL_GetTick();
  control_loop_next_tick = imu_last_tick + CONTROL_LOOP_PERIOD_MS;
  control_loop_schedule_late_count = 0U;
  control_loop_schedule_resync_count = 0U;
  control_loop_last_lateness_ms = 0U;
  imu_control_roll_deg = 0.0f;
  imu_control_pitch_deg = 0.0f;
  imu_read_fail_streak = 0U;
  imu_read_coast_count = 0U;
  imu_init_start_tick = HAL_GetTick();
  if (imu_init_start_tick < MPU_POWER_UP_DELAY_MS)
  {
    imu_init_extra_delay_ms =
        (uint32_t)(MPU_POWER_UP_DELAY_MS - imu_init_start_tick);
    HAL_Delay(imu_init_extra_delay_ms);
  }
  else
  {
    imu_init_extra_delay_ms = 0U;
  }
  mpu_status = MPU6500_Init(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    {
      main_loop_pass_count++;
      Watchdog_Refresh();
      IBUS_Receiver_Update(HAL_GetTick());
      RC_Input_Update();

      EscCalibration_Update();
      if (esc_calibration_active != 0U)
      {
        continue;
      }
      rc_command_system_ready = RC_CommandSystemReady();
      RC_Arm_Update(rc_command_system_ready);
      if ((flight_stop_session_active != 0U) &&
          (rc_arm_request == 0U))
      {
        FlightStopDiagnostics_Latch(FlightStopDiagnostics_RCReason());
      }

      if (rc_input_valid != 0U)
      {
        roll_setpoint_deg = rc_input_roll_deg;
        pitch_setpoint_deg = rc_input_pitch_deg;
        yaw_rate_setpoint_dps = rc_input_yaw_rate_dps;
        rc_command_throttle_preview_us =
            MotorControl_ThrottleFromNormalized(rc_input_throttle_normalized);
        rc_command_preview_valid = 1U;
      }
      else
      {
        roll_setpoint_deg = 0.0f;
        pitch_setpoint_deg = 0.0f;
        yaw_rate_setpoint_dps = 0.0f;
        rc_command_throttle_preview_us = MOTOR_CONTROL_IDLE_US;
        rc_command_preview_valid = 0U;
      }

#if RC_COMMAND_MOTOR_OUTPUT_ENABLED
      motor_throttle_us = rc_command_throttle_preview_us;
      motor_arm_request = rc_arm_request;
#else
      motor_throttle_us = MOTOR_CONTROL_IDLE_US;
      motor_arm_request = 0U;
      MotorControl_ForceDisarm();
#endif
    }

    if (mpu_status == HAL_OK)
    {
      /* Gyro calibration once at startup; the drone must be still. */
      if (imu_filter_ready == 0U)
      {
        imu_calibration_status = IMU_CalibrateGyro(500U);
        if (imu_calibration_status == HAL_OK)
        {
          imu_filter_ready = 1U;
          imu_last_tick = HAL_GetTick();
          control_loop_next_tick =
              imu_last_tick + CONTROL_LOOP_PERIOD_MS;
          PID_Controller_Reset(&roll_pid);
          PID_Controller_Reset(&pitch_pid);
          PID_Controller_Reset(&yaw_rate_pid);
          RC_CommandForceSystemDisarm();
          Watchdog_Start();
        }
        else if (imu_calibration_status != HAL_BUSY)
        {
          mpu_status = imu_calibration_status;
        }
      }
      else
      {
        uint32_t now_tick;
        float dt_s;

        /* 100 Hz control step */
        ControlLoop_WaitNext();
        mpu_status = MPU6500_ReadAll(&hi2c1, &mpu_data);
        if (mpu_status == HAL_OK)
        {
          imu_read_fail_streak = 0U;
          now_tick = HAL_GetTick();
          dt_s = (float)(now_tick - imu_last_tick) * 0.001f;
          imu_last_tick = now_tick;
          if ((dt_s > 0.0f) && (dt_s <= 0.1f))
          {
            IMU_Filter_Update(&imu_filter, &mpu_data, dt_s);
            IMU_SignCheck_Update();
            imu_control_roll_deg =
                imu_filter.roll_deg - IMU_LEVEL_ROLL_OFFSET_DEG;
            imu_control_pitch_deg =
                imu_filter.pitch_deg - IMU_LEVEL_PITCH_OFFSET_DEG;
            if (motor_armed == 0U)
            {
              PID_Controller_ResetIntegral(&roll_pid);
              PID_Controller_ResetIntegral(&pitch_pid);
              PID_Controller_ResetIntegral(&yaw_rate_pid);
            }
            {
              uint8_t integral_enabled =
                  (motor_applied_throttle_us >
                   PID_INTEGRAL_ENABLE_THROTTLE_US) ? 1U : 0U;

              PID_Controller_SetIntegralEnabled(&roll_pid, integral_enabled);
              PID_Controller_SetIntegralEnabled(&pitch_pid, integral_enabled);
              PID_Controller_SetIntegralEnabled(&yaw_rate_pid,
                                                integral_enabled);
            }
            roll_pid_output = PID_Controller_Update(&roll_pid,
                                                    roll_setpoint_deg,
                                                    imu_control_roll_deg,
                                                    imu_filter.gyro_x_dps,
                                                    dt_s);
            pitch_pid_output = PID_Controller_Update(&pitch_pid,
                                                     pitch_setpoint_deg,
                                                     imu_control_pitch_deg,
                                                     imu_filter.gyro_y_dps,
                                                     dt_s);
            yaw_pid_output = PID_Controller_Update(&yaw_rate_pid,
                                                   yaw_rate_setpoint_dps,
                                                   imu_filter.gyro_z_dps,
                                                   0.0f,
                                                   dt_s);
            MotorControl_Update(mpu_status,
                                imu_filter_ready,
                                imu_control_roll_deg,
                                imu_control_pitch_deg,
                                roll_pid_output,
                                pitch_pid_output,
                                yaw_pid_output);
            if (motor_armed != 0U)
            {
              if (flight_yaw_session_active == 0U)
              {
                flight_yaw_session_active = 1U;
                flight_yaw_armed_loop_count = 0U;
                flight_yaw_saturated_count = 0U;
                flight_yaw_integral_peak = 0.0f;
                flight_yaw_rate_mean_dps = 0.0f;
                flight_yaw_output_mean_us = 0.0f;
                flight_roll_integral_peak = 0.0f;
                flight_pitch_integral_peak = 0.0f;
              }
              {
                float axis_integral_abs;

                axis_integral_abs = fabsf(roll_pid.integral);
                if (axis_integral_abs > flight_roll_integral_peak)
                {
                  flight_roll_integral_peak = axis_integral_abs;
                }
                axis_integral_abs = fabsf(pitch_pid.integral);
                if (axis_integral_abs > flight_pitch_integral_peak)
                {
                  flight_pitch_integral_peak = axis_integral_abs;
                }
              }
              if (fabsf(yaw_rate_setpoint_dps) <=
                  FLIGHT_YAW_CENTERED_SETPOINT_DPS)
              {
                float yaw_integral_abs;

                flight_yaw_armed_loop_count++;
                flight_yaw_rate_mean_dps +=
                    (imu_filter.gyro_z_dps - flight_yaw_rate_mean_dps) /
                    (float)flight_yaw_armed_loop_count;
                flight_yaw_output_mean_us +=
                    (yaw_pid_output - flight_yaw_output_mean_us) /
                    (float)flight_yaw_armed_loop_count;
                yaw_integral_abs = fabsf(yaw_rate_pid.integral);
                if (yaw_integral_abs > flight_yaw_integral_peak)
                {
                  flight_yaw_integral_peak = yaw_integral_abs;
                }
                if (fabsf(yaw_pid_output) >=
                    (yaw_rate_pid.output_limit -
                     FLIGHT_YAW_SATURATION_EPSILON_US))
                {
                  flight_yaw_saturated_count++;
                }
              }
            }
            else
            {
              flight_yaw_session_active = 0U;
            }
            if (motor_armed != 0U)
            {
              flight_stop_session_active = 1U;
            }
            else if ((flight_stop_session_active != 0U) &&
                     (rc_arm_request != 0U))
            {
              FlightStopDiagnostics_Latch(
                  FLIGHT_STOP_REASON_MOTOR_CONTROL_BLOCKED);
            }
            if ((rc_arm_request != 0U) && (motor_armed == 0U))
            {
              RC_Arm_ForceSystemBlock();
            }
            FlightDiagnostics_Update(dt_s, &mpu_data, motor_armed);
          }
          else
          {
            FlightStopDiagnostics_Latch(
                FLIGHT_STOP_REASON_LOOP_TIMING_ERROR);
            RC_CommandForceSystemDisarm();
          }
        }
        else
        {
          imu_read_fail_streak++;
          if (imu_read_fail_streak >= IMU_READ_FAIL_TOLERANCE)
          {
            FlightStopDiagnostics_Latch(FLIGHT_STOP_REASON_MPU_READ_ERROR);
            RC_CommandForceSystemDisarm();
          }
          else
          {
            imu_read_coast_count++;
            MotorControl_Update(HAL_OK,
                                imu_filter_ready,
                                imu_control_roll_deg,
                                imu_control_pitch_deg,
                                roll_pid_output,
                                pitch_pid_output,
                                yaw_pid_output);
            mpu_status = HAL_OK;
          }
        }
      }
    }
    else
    {
      FlightStopDiagnostics_Latch(FLIGHT_STOP_REASON_MPU_READ_ERROR);
      RC_CommandForceSystemDisarm();
      /* IMU error: motors stop and the sensor is re-initialized. */
      imu_recovery_pass_count++;
      Watchdog_Refresh();
      HAL_Delay(20);
#if MPU6500_USE_SPI
      mpu_status = MPU6500_Init(&hi2c1);
#else
      mpu_status = I2C1_BusRecover();
      if (mpu_status == HAL_OK)
      {
        mpu_status = MPU6500_Init(&hi2c1);
      }
#endif
      imu_read_fail_streak = 0U;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_2);
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_3);
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_4);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MPU6500_CS_Pin */
  GPIO_InitStruct.Pin = MPU6500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU6500_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU6500_SCK_Pin MPU6500_MISO_Pin MPU6500_MOSI_Pin */
  GPIO_InitStruct.Pin = MPU6500_SCK_Pin|MPU6500_MISO_Pin|MPU6500_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  ESC_PWM_SetMinimum();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
