#pragma once

#include "stm32l4xx_hal.h"
#include "i2c_dma_manager.h"
#include "cmsis_os.h"
#include "common_types.h"

// Definicja adresu I2C (7-bitowy adres przesunięty w lewo)
#define MPU6500_I2C_ADDR (0x68 << 1)

// Definicje rejestrów MPU-6500
#define MPU6500_WHO_AM_I        0x75
#define MPU6500_WHO_AM_I_VAL    0x70
#define MPU6500_PWR_MGMT_1      0x6B
#define MPU6500_GYRO_CONFIG     0x1B
#define MPU6500_ACCEL_CONFIG    0x1C
#define MPU6500_ACCEL_XOUT_H    0x3B

// Progi dla autotestu
#define SELF_TEST_CHANGE_THRESHOLD 500


// Enum dla wyników autotestu (jeśli go używasz)
typedef enum {
    MPU_TEST_SUCCESS = 0,
    MPU_TEST_ERR_READ_BASE = -2,
    MPU_TEST_ERR_READ_ST = -3,
    MPU_TEST_ERR_ACCEL_FAIL = -4,
    MPU_TEST_ERR_GYRO_FAIL = -5
} MPU_Test_Result_t;

// Prototypy funkcji
HAL_StatusTypeDef mpu6500_read_raw_data(I2C_HandleTypeDef* hi2c, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
HAL_StatusTypeDef mpu6500_calculate_orientation_offsets(I2C_HandleTypeDef* hi2c, int num_samples, float* offset_pitch, float* offset_roll);

HAL_StatusTypeDef mpu6500_init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef mpu6500_orientation_rad(I2C_HandleTypeDef* hi2c, float* final_pitch_rad, float* final_roll_rad, float* acc_mps2);
// Jeśli używasz, dodaj też prototyp dla self_test
// MPU_Test_Result_t mpu6500_run_self_test(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef mpu6500_init_DMA(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef mpu6500_read_raw_data_DMA(I2C_HandleTypeDef* hi2c, osThreadId taskHandle, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
