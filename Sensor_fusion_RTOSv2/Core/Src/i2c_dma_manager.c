#include "i2c_dma_manager.h"
#include "task.h"

static osMutexId g_i2c_bus_mutex;

// --- DEFINICJA ZMIENNEJ GLOBALNEJ ---
volatile osThreadId task_waiting_for_i2c1_dma = NULL;

void I2C_DMA_Manager_Init(void) {
    osMutexDef(i2c_bus_mutex);
    g_i2c_bus_mutex = osMutexCreate(osMutex(i2c_bus_mutex));
}


HAL_StatusTypeDef I2C_DMA_Read(I2C_DMA_Transaction_t* transaction, uint16_t device_addr, uint16_t mem_addr) {
    if (osMutexWait(g_i2c_bus_mutex, osWaitForever) != osOK) {
        return HAL_ERROR;
    }

    if (transaction->hi2c->Instance == I2C1) {
        task_waiting_for_i2c1_dma = transaction->task_to_notify;
    }

    transaction->status = HAL_BUSY;
    ulTaskNotifyTake(pdTRUE, 0);

    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_DMA(
        transaction->hi2c, device_addr, mem_addr,
        I2C_MEMADD_SIZE_8BIT, (uint8_t*)transaction->rx_buffer, transaction->rx_size
    );

    if (hal_status != HAL_OK) {
        task_waiting_for_i2c1_dma = NULL;
        osMutexRelease(g_i2c_bus_mutex);
        return hal_status;
    }


    uint32_t notification_value = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

    if (notification_value > 0) {
        // Status błędu/sukcesu zostanie ustawiony w callbacku
    } else {
        printf("ERROR: Timeout DMA!\n");
        hal_status = HAL_TIMEOUT;
        HAL_I2C_Master_Abort_IT(transaction->hi2c, device_addr);
    }

    task_waiting_for_i2c1_dma = NULL;
    osMutexRelease(g_i2c_bus_mutex);

    return hal_status;
}


