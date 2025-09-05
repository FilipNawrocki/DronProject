#ifndef I2C_DMA_MANAGER_H_
#define I2C_DMA_MANAGER_H_

#include "main.h"
#include "cmsis_os.h"

typedef struct {
    I2C_HandleTypeDef* hi2c;
    osThreadId task_to_notify;
    volatile uint8_t* rx_buffer;
    uint16_t rx_size;
    volatile HAL_StatusTypeDef status;
} I2C_DMA_Transaction_t;

// --- UŻYJEMY BEZPIECZNIEJSZEGO PODEJŚCIA ---
// Zamiast udostępniać wskaźnik do całej transakcji, udostępnimy
// tylko uchwyt do zadania, które czeka na daną magistralę.
extern volatile osThreadId task_waiting_for_i2c1_dma;

void I2C_DMA_Manager_Init(void);
HAL_StatusTypeDef I2C_DMA_Read(I2C_DMA_Transaction_t* transaction, uint16_t device_addr, uint16_t mem_addr);


#endif /* I2C_DMA_MANAGER_H_ */
