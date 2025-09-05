#include "Ips25hb.h"
#include "i2c.h"
#include <stdbool.h>
#include "i2c_dma_manager.h"
#include "math.h"


static void lps_write_reg(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(hi2c, LPS25HB_ADDR, reg, 1, &value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef  pressureToAltitude(float pressure_hPa, float* altitude_out) {
    if (pressure_hPa <= 0) {
    	return HAL_ERROR;
    }
    float base = pressure_hPa / 1013.25f;
    float term = expf(0.190295f * logf(base));
    float temp = 1.0f - term;
    float altitiude =  44330.0f * temp;
    *altitude_out = altitiude;

    return HAL_OK; // Zwróć sukces
}

float lps25hb_read_pressure(I2C_HandleTypeDef* hi2c)
{
     int32_t pressure = 0;

     uint8_t buffer[3];
         HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, LPS25HB_ADDR, LPS25HB_PRESS_OUT_XL | 0x80, I2C_MEMADD_SIZE_8BIT, buffer, 3, 100);

         if (status != HAL_OK) {
             printf("LPS25HB: Pressure read failed (status: %d)\n", status);
             HAL_I2C_DeInit(hi2c);
             HAL_Delay(5); // Krótka pauza
             MX_I2C1_Init();
             return -1.0f;
         }
         int32_t raw_pressure = (int32_t)((buffer[2] << 16) | (buffer[1] << 8) | buffer[0]);


         return (float)raw_pressure / 4096.0f;
}


HAL_StatusTypeDef lps25hb_init_DMA(I2C_HandleTypeDef* hi2c)
{
	uint8_t who_am_i;
	HAL_StatusTypeDef status;

	    status = HAL_I2C_IsDeviceReady(hi2c, LPS25HB_ADDR, 2, 100);
	    if (status != HAL_OK) {
	        printf("LPS25HB: Device not ready (status: %d)\n", status);
	        return HAL_ERROR;
	    }

	    status = HAL_I2C_Mem_Read(hi2c, LPS25HB_ADDR, LPS25HB_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100);
	    if (status != HAL_OK) {
	        printf("LPS25HB: WHO_AM_I read failed (status: %d)\n", status);
	        return HAL_ERROR;
	    }

	    if (who_am_i != 0xbd) {
	        printf("LPS25HB: Wrong WHO_AM_I value: 0x%X\n", who_am_i);
	        return HAL_ERROR;
	    }

	lps_write_reg(hi2c, LPS25HB_CTRL_REG1,  0xC0);
	lps_write_reg(hi2c, LPS25HB_CTRL_REG2,  0x40);
	lps_write_reg(hi2c, LPS25HB_FIFO_CTRL,  0xDF);

	return HAL_OK;
}

/*
 	 NIEDZIAŁA CHUJ WIE CZEMU

HAL_StatusTypeDef lps25hb_read_pressure_DMA(I2C_HandleTypeDef* hi2c, osThreadId taskHandle, uint32_t* pressure){
    // 1. Przygotuj strukturę transakcji
    I2C_DMA_Transaction_t transaction = {
        .hi2c = hi2c,
		.task_to_notify = taskHandle,
        .rx_buffer = lps_buffer,
        .rx_size = 3
    };

    // 2. Wywołaj generyczną funkcję odczytu z menedżera
    HAL_StatusTypeDef status = I2C_DMA_Read(&transaction, LPS25HB_ADDR, LPS25HB_PRESS_OUT_XL);

    if (status == HAL_OK) {
        // 3. Przetwórz dane, jeśli odczyt się powiódł
        *pressure = (uint32_t)((lps_buffer[2] << 16) | (lps_buffer[1] << 8) | lps_buffer[0]);
    }

    return status;
} */

