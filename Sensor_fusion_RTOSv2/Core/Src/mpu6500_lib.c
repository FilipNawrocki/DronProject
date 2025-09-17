/*
 * mpu6500_lib.c
 *
 *  Created on: Jul 7, 2025
 *      Author: 48791
 */

#include <efk_orientation.h>
#include "mpu6500_lib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "i2c_dma_manager.h"

static float offset_pitch, offset_roll;
static KalmanRollPich orientation_ekf;

extern osMutexId g_i2c_bus_mutex;


HAL_StatusTypeDef mpu6500_calibrate_gyro_bias(I2C_HandleTypeDef* hi2c, float* gyro_bias,  int num_samples) {
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t ax, ay, az, gx, gy, gz;
    for (int i = 0; i < num_samples; i++) {
        if (mpu6500_read_raw_data(hi2c, &ax, &ay, &az, &gx, &gy, &gz) != HAL_OK) {
            return HAL_ERROR;
        }
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        HAL_Delay(5);
    }
    float gyro_bias_x = ((float)gx_sum / num_samples) / 131.0f * (M_PI / 180.0f);
    float gyro_bias_y = ((float)gy_sum / num_samples) / 131.0f * (M_PI / 180.0f);
    float gyro_bias_z = ((float)gz_sum / num_samples) / 131.0f * (M_PI / 180.0f);

    gyro_bias[0] = gyro_bias_x;
    gyro_bias[1] = gyro_bias_y;
    gyro_bias[2] = gyro_bias_z;



    return HAL_OK;
}

HAL_StatusTypeDef mpu6500_calculate_orientation_offsets(I2C_HandleTypeDef* hi2c, int num_samples, float* offset_pitch, float* offset_roll)
{
    long acc_x_sum = 0;
    long acc_y_sum = 0;
    long acc_z_sum = 0;
    int16_t raw_ax, raw_ay, raw_az, dummy_gx, dummy_gy, dummy_gz;
    printf("Kalibracja orientacji... Nie ruszaj czujnika przez 2 sekundy.\n");

    for (int i = 0; i < num_samples; i++) {
        if (mpu6500_read_raw_data(hi2c, &raw_ax, &raw_ay, &raw_az, &dummy_gx, &dummy_gy, &dummy_gz) != HAL_OK) {
            printf("Blad odczytu danych podczas kalibracji offsetu.\n");
            return HAL_ERROR;
        }
        acc_x_sum += raw_ax;
        acc_y_sum += raw_ay;
        acc_z_sum += raw_az;
        HAL_Delay(10);
    }
    float avg_ax = (float)acc_x_sum / num_samples;
    float avg_ay = (float)acc_y_sum / num_samples;
    float avg_az = (float)acc_z_sum / num_samples;

    *offset_roll = atan2f(avg_ay, sqrtf(avg_ax * avg_ax + avg_az * avg_az));
    *offset_pitch = atan2f(-avg_ax, sqrtf(avg_ay * avg_ay + avg_az * avg_az));



    return HAL_OK;
}

HAL_StatusTypeDef mpu6500_read_raw_data(I2C_HandleTypeDef* hi2c, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buffer[14];

    if (HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 14, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    *az = (int16_t)((buffer[4] << 8) | buffer[5]);

    *gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    *gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    *gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    return HAL_OK;
}


float pitch_gyro = 0.0f;
float roll_gyro = 0.0f;
float yaw_gyro = 0.0f;

float pitch_accel = 0.0f;
float roll_accel = 0.0f;

static uint32_t last_update_time_raw = 0;

void mpu6500_update_orientation_raw(I2C_HandleTypeDef* hi2c) {
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

    uint32_t current_time = HAL_GetTick();
    if (last_update_time_raw == 0) {
        last_update_time_raw = current_time;
        return;
    }
    float dt = (float)(current_time - last_update_time_raw) / 1000.0f;
    last_update_time_raw = current_time;

    if (mpu6500_read_raw_data(hi2c, &raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz) != HAL_OK) {
        return;
    }

    float acc_x = (float)raw_ax / 16384.0f;
    float acc_y = (float)raw_ay / 16384.0f;
    float acc_z = (float)raw_az / 16384.0f;
    float gyro_x = (float)raw_gx / 131.0f;
    float gyro_y = (float)raw_gy / 131.0f;
    float gyro_z = (float)raw_gz / 131.0f;

    pitch_accel = atan2f(acc_y, acc_z) * 180.0f / M_PI;
    roll_accel = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / M_PI;

    pitch_gyro += gyro_x * dt;
    roll_gyro += gyro_y * dt;
    yaw_gyro += gyro_z * dt;
}
/*
HAL_StatusTypeDef mpu6500_orientation_rad(I2C_HandleTypeDef* hi2c, float* final_pitch_rad, float* final_roll_rad, float* acc_mps2){

	uint32_t current_time = HAL_GetTick();
	    if (last_update_time_raw == 0) {
	        last_update_time_raw = current_time;
	        return;
	    }
	    float dt = (float)(current_time - last_update_time_raw) / 1000.0f;
	    last_update_time_raw = current_time;

	// 1. Odczyt surowych danych
	int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
	if (mpu6500_read_raw_data(hi2c, &raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz) != HAL_OK) {
	      printf("Błąd odczytu MPU!\n");
	      return;
	}

		        // 2. Przeliczenie na jednostki fizyczne
		        acc_mps2[0] = (float)raw_ax / 16384.0f * G_CONST;
		        acc_mps2[1] = (float)raw_ay / 16384.0f * G_CONST;
				acc_mps2[2] = (float)raw_az / 16384.0f * G_CONST;

		        // UWAGA: Sprawdź ustawienia zakresu żyroskopu! 131.0f jest dla ±250dps
		        float gyr_rps[3]  = {
		            (float)raw_gx / 131.0f * (M_PI / 180.0f),
		            (float)raw_gy / 131.0f * (M_PI / 180.0f),
		            (float)raw_gz / 131.0f * (M_PI / 180.0f)
		        };

		        // 3. Aktualizacja filtru Kalmana
	KalmanRollPich_Predict(&orientation_ekf, gyr_rps, dt);
	KalmanRollPich_Update(&orientation_ekf, acc_mps2);

	// 4. Wyświetlanie wyników
	*final_pitch_rad = orientation_ekf.theta_rad;
	*final_roll_rad = orientation_ekf.phi_rad;

	return HAL_OK;
}
*/
//         --------DMA-------
static volatile uint8_t mpu_buffer[14];


HAL_StatusTypeDef mpu6500_init_DMA(I2C_HandleTypeDef* hi2c){

    uint8_t who_am_i_val;
    uint8_t data;

    if (HAL_I2C_IsDeviceReady(hi2c, MPU6500_I2C_ADDR, 2, 100) != HAL_OK) {
        printf("ERROR: Urzedzenie MPU-6500 nie odpowiada.\n");
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR, MPU6500_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i_val, 1, 100) != HAL_OK) {
        printf("ERROR: Nie można odczytac rejestru WHO_AM_I.\n");
        return HAL_ERROR;
    }

    if (who_am_i_val != MPU6500_WHO_AM_I_VAL) {
        printf("ERROR: Zly ID urzadzenia. ID: 0x%X (oczekiwano 0x%X)\n", who_am_i_val, MPU6500_WHO_AM_I_VAL);
        return HAL_ERROR;
    }

    data = 0x00; // Wybudzenie urządzenia
    if (HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(100);
    uint8_t config_val = 3;
    HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &config_val, 1, 100);

    uint8_t accel_config2_val = 3;
    HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR, 0x1D, I2C_MEMADD_SIZE_8BIT, &accel_config2_val, 1, 100);

    return HAL_OK;
}
/*
 *
 	 NIE WIEM CZY DZAIAŁA ALE NIE UŻYWAM
HAL_StatusTypeDef mpu6500_read_raw_data_DMA(I2C_HandleTypeDef* hi2c, osThreadId taskHandle, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
    I2C_DMA_Transaction_t transaction = {
        .hi2c = hi2c,
		.task_to_notify = taskHandle,
        .rx_buffer = mpu_buffer,
        .rx_size = 14
    };
    HAL_StatusTypeDef status = I2C_DMA_Read(&transaction, MPU6500_I2C_ADDR, MPU6500_ACCEL_XOUT_H);
	if(status == HAL_OK){
		*ax = (int16_t)((mpu_buffer[0] << 8) | mpu_buffer[1]);
		*ay = (int16_t)((mpu_buffer[2] << 8) | mpu_buffer[3]);
		*az = (int16_t)((mpu_buffer[4] << 8) | mpu_buffer[5]);

		*gx = (int16_t)((mpu_buffer[8] << 8) | mpu_buffer[9]);
		*gy = (int16_t)((mpu_buffer[10] << 8) | mpu_buffer[11]);
		*gz = (int16_t)((mpu_buffer[12] << 8) | mpu_buffer[13]);

	}

	return status;
}
*/


