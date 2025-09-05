#pragma once
#include "stm32l4xx.h"
#include "cmsis_os.h"

#define LPS25HB_ADDR        0xBA

#define LPS25HB_WHO_AM_I 		0x0F
#define LPS25HB_CTRL_REG1 		0x20
#define LPS25HB_CTRL_REG2 		0x21
#define LPS25HB_CTRL_REG3 		0x22
#define LPS25HB_CTRL_REG4 		0x23
#define LPS25HB_PRESS_OUT_XL 	0x28
#define LPS25HB_PRESS_OUT_L 	0x29
#define LPS25HB_PRESS_OUT_H 	0x2A
#define LPS25HB_TEMP_OUT_L 		0x2B
#define LPS25HB_TEMP_OUT_H 		0x2C
#define LPS25HB_RPDS_L 			0x39
#define LPS25HB_RPDS_H 			0x3A
#define LPS25HB_FIFO_CTRL		0x2E

#define TIMEOUT  100


HAL_StatusTypeDef lps25hb_init_DMA(I2C_HandleTypeDef* hi2c);
float lps25hb_read_pressure(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef pressureToAltitude(float pressure_hPa, float* altitude_out);
//HAL_StatusTypeDef lps25hb_read_pressure_DMA(I2C_HandleTypeDef* hi2c, osThreadId taskHandle, uint32_t* pressure);
