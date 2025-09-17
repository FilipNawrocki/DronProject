/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "math.h"
#include "i2c_dma_manager.h"
#include "mpu6500_lib.h"
#include "Ips25hb.h"
#include "common_types.h"
#include "efk_orientation.h"
#include "ekf_altitude.h"
#include "control_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Globalne uchwyty z CubeMX
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart5;
extern osThreadId defaultTaskHandle;

const float ACCEL_SENSITIVITY = 16384.0f;
const float GYRO_SENSITIVITY = 131.0f;
const float G_CONST = 9.81f;

//Do usunięcia
volatile float g_debug_pitch_deg = 0.0f;
volatile float g_debug_roll_deg = 0.0f;
volatile float g_debug_altitude = 0.0f;
volatile float debug_pwm1 = 0.0f;
volatile float debug_pwm2 = 0.0f;
volatile float debug_pwm3 = 0.0f;
volatile float debug_pwm4 = 0.0f;
volatile float pitch_error_new = 0.0f;
volatile float pitch_error = 0.0f;

/* USER CODE END Variables */
osThreadId SensorTaskHandle;
osThreadId Fusion_Sensor_EHandle;
osThreadId DiagnosticTaskHandle;
osThreadId Control_TaskHandle;
osThreadId ComunicationTasHandle;
osMessageQId rawSensorQueueHandle;
osMessageQId orientationQueueHandle;
osMessageQId UartDMADataQueueHandle;
osSemaphoreId initDoneSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartDiagnosticTask(void const * argument);
void StartControl_Task(void const * argument);
void StartComunicationTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	I2C_DMA_Manager_Init();



  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of initDoneSem */
  osSemaphoreDef(initDoneSem);
  initDoneSemHandle = osSemaphoreCreate(osSemaphore(initDoneSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  osSemaphoreWait(initDoneSemHandle, 0);

  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of rawSensorQueue */
  osMessageQDef(rawSensorQueue, 5, uint32_t);
  rawSensorQueueHandle = osMessageCreate(osMessageQ(rawSensorQueue), NULL);

  /* definition and creation of orientationQueue */
  osMessageQDef(orientationQueue, 5, uint32_t);
  orientationQueueHandle = osMessageCreate(osMessageQ(orientationQueue), NULL);

  /* definition and creation of UartDMADataQueue */
  osMessageQDef(UartDMADataQueue, 1, uint32_t);
  UartDMADataQueueHandle = osMessageCreate(osMessageQ(UartDMADataQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* definition and creation of Fusion_Sensor_E */
  osThreadDef(Fusion_Sensor_E, StartTask02, osPriorityAboveNormal, 0, 4096);
  Fusion_Sensor_EHandle = osThreadCreate(osThread(Fusion_Sensor_E), NULL);

  /* definition and creation of DiagnosticTask */
  osThreadDef(DiagnosticTask, StartDiagnosticTask, osPriorityLow, 0, 512);
  DiagnosticTaskHandle = osThreadCreate(osThread(DiagnosticTask), NULL);

  /* definition and creation of Control_Task */
  osThreadDef(Control_Task, StartControl_Task, osPriorityAboveNormal, 0, 1024);
  Control_TaskHandle = osThreadCreate(osThread(Control_Task), NULL);

  /* definition and creation of ComunicationTas */
  osThreadDef(ComunicationTas, StartComunicationTask, osPriorityBelowNormal, 0, 256);
  ComunicationTasHandle = osThreadCreate(osThread(ComunicationTas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    osSemaphoreWait(initDoneSemHandle, osWaitForever);

    static volatile uint8_t mpu_dma_buffer[16];
    static volatile uint8_t lps_dma_buffer[8];

    I2C_DMA_Transaction_t mpu_transaction;
    mpu_transaction.hi2c = &hi2c1;
    mpu_transaction.task_to_notify = SensorTaskHandle;
    mpu_transaction.rx_buffer = mpu_dma_buffer;
    mpu_transaction.rx_size = 14;

    I2C_DMA_Transaction_t lps_transaction;
    lps_transaction.hi2c = &hi2c1;
    lps_transaction.task_to_notify = SensorTaskHandle;
    lps_transaction.rx_buffer = lps_dma_buffer;
    lps_transaction.rx_size = 3;

    osDelay(50);
    uint8_t i = 4;
    static RawSensorData_t sensor_data_pocket;
  /* Infinite loop */
  for(;;)
  {
	    sensor_data_pocket.baro_data_updated = 0;
	    HAL_StatusTypeDef mpu_status, lps_status;

	    // --- Odczyt MPU6500 z pętlą ponawiania próby ---
	    const int MAX_RETRIES = 3;
	    int retries = 0;

	    do {
	        mpu_status = I2C_DMA_Read(&mpu_transaction, MPU6500_I2C_ADDR, MPU6500_ACCEL_XOUT_H);
	        if (mpu_status != HAL_OK) {
	            retries++;
	            osDelay(5);
	        }
	    } while (mpu_status != HAL_OK && retries < MAX_RETRIES);


        if (mpu_status == HAL_OK) {
            sensor_data_pocket.timestamp_mpu = osKernelSysTick();
            sensor_data_pocket.ax = (int16_t)((mpu_dma_buffer[0] << 8) | mpu_dma_buffer[1]);
            sensor_data_pocket.ay = (int16_t)((mpu_dma_buffer[2] << 8) | mpu_dma_buffer[3]);
            sensor_data_pocket.az = (int16_t)((mpu_dma_buffer[4] << 8) | mpu_dma_buffer[5]);
            sensor_data_pocket.gx = (int16_t)((mpu_dma_buffer[8] << 8) | mpu_dma_buffer[9]);
            sensor_data_pocket.gy = (int16_t)((mpu_dma_buffer[10] << 8) | mpu_dma_buffer[11]);
            sensor_data_pocket.gz = (int16_t)((mpu_dma_buffer[12] << 8) | mpu_dma_buffer[13]);
        } else {
        	HAL_I2C_DeInit(mpu_transaction.hi2c);
            osDelay(10);
            HAL_I2C_Init(mpu_transaction.hi2c);
            osDelay(5);
            continue;
        }

      if(i >= 4){
          if (I2C_DMA_Read(&lps_transaction, LPS25HB_ADDR, LPS25HB_PRESS_OUT_XL | 0x80) == HAL_OK) {
              sensor_data_pocket.pressure_raw = (uint32_t)((lps_dma_buffer[2] << 16) | (lps_dma_buffer[1] << 8) | lps_dma_buffer[0]);
              sensor_data_pocket.baro_data_updated = 1;
          } else {
              // printf("LPS FAIL -> Blad odczytu DMA.\n");
          }
          i = 0;
      }
      i++;

      osMessagePut(rawSensorQueueHandle, (uint32_t)&sensor_data_pocket, 10);
      osDelay(4);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Fusion_Sensor_E thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
    static float gyro_bias[3];
    static float roll_offset, pitch_offset;
    static KalmanRollPich orientation_ekf;
    static EKF_Altitude_t altitude_ekf;
    static float init_alti;

    static RawSensorData_t received_raw_data;
    static OrientationData_t processed_data;
    static uint32_t last_predict_time = 0;

    if (mpu6500_init_DMA(&hi2c1) != HAL_OK) {
        printf("KRYTYCZNY BLAD: Inicjalizacja MPU-6500\n");
        Error_Handler();
    }
    /*
    if (lps25hb_init_DMA(&hi2c1) != HAL_OK) {
        printf("KRYTYCZNY BLAD: Inicjalizacja LPS25HB\n");
        Error_Handler();
    }
    */
    if (mpu6500_calculate_orientation_offsets(&hi2c1, 500, &pitch_offset, &roll_offset) != HAL_OK) {
        Error_Handler();
    }
    if (mpu6500_calibrate_gyro_bias(&hi2c1, gyro_bias, 500) != HAL_OK) {
        Error_Handler();
    }
    /*
    if (pressureToAltitude(lps25hb_read_pressure(&hi2c1), &init_alti) != HAL_OK) {
        printf("Blad obliczania wysokosci.\n");
        Error_Handler();
    }
	*/
    float Pinit = 1.0f;
    float Q[2] = {0.001f, 0.001f};
    float R[2] = {0.03f, 0.03f};
    KalmanRollPich_init(&orientation_ekf, gyro_bias, roll_offset, pitch_offset, Pinit, Q, R);
    //EKF_Altitude_Init(&altitude_ekf, init_alti);

    osSemaphoreRelease(initDoneSemHandle);
    osEvent event;

  /* Infinite loop */
  for(;;)
  {
	event = osMessageGet(rawSensorQueueHandle, osWaitForever);


        if(event.status == osEventMessage)
        {
            memcpy(&received_raw_data, (void*)event.value.p, sizeof(RawSensorData_t));

            float dt_mpu;
            if (last_predict_time == 0) {
                dt_mpu = 0.01f;
            } else {
                dt_mpu = (float)(received_raw_data.timestamp_mpu - last_predict_time) / 1000.0f;
            }
            last_predict_time = received_raw_data.timestamp_mpu;

            float acc_mps2[3] = {
                    (float)received_raw_data.ax / ACCEL_SENSITIVITY * G_CONST,
                    (float)received_raw_data.ay / ACCEL_SENSITIVITY * G_CONST,
                    (float)received_raw_data.az / ACCEL_SENSITIVITY * G_CONST
                };
            float gyr_rps[3] = {
                    (float)received_raw_data.gx / GYRO_SENSITIVITY * (M_PI / 180.0f),
                    (float)received_raw_data.gy / GYRO_SENSITIVITY * (M_PI / 180.0f),
                    (float)received_raw_data.gz / GYRO_SENSITIVITY * (M_PI / 180.0f)
                };

            KalmanRollPich_Predict(&orientation_ekf, gyr_rps, dt_mpu);
            KalmanRollPich_Update(&orientation_ekf, acc_mps2);

      	    float acc_z_world = - acc_mps2[0] * sin(orientation_ekf.theta_rad) +
      	                      acc_mps2[1] * sin(orientation_ekf.phi_rad) * cos(orientation_ekf.theta_rad) +
      	                      acc_mps2[2] * cos(orientation_ekf.phi_rad) * cos(orientation_ekf.theta_rad);



      	    float acc_z_corrected = acc_z_world - 9.81f;

      	    EKF_Altitude_Predict(&altitude_ekf, acc_z_corrected, dt_mpu);

      	    if(received_raw_data.baro_data_updated == 1){
      	    	float baro_altitude;
      	    	float baro_pressure = received_raw_data.pressure_raw / 4096.0f;
      	    	pressureToAltitude(baro_pressure, &baro_altitude);
      	    	EKF_Altitude_Update(&altitude_ekf, baro_altitude);
      	    }

            processed_data.pitch_rad = orientation_ekf.theta_rad;
            processed_data.roll_rad = orientation_ekf.phi_rad;
            processed_data.altitude = -altitude_ekf.x[0] - init_alti;
            processed_data.gx_rps= orientation_ekf.p;
            processed_data.gy_rps= orientation_ekf.q;
            processed_data.gz_rps= orientation_ekf.r;

            g_debug_pitch_deg = processed_data.pitch_rad * 180.0f / M_PI;
            g_debug_roll_deg = processed_data.roll_rad * 180.0f / M_PI;
            g_debug_altitude = processed_data.altitude;

            osMessagePut(orientationQueueHandle, (uint32_t)&processed_data, 10);
        }

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartDiagnosticTask */
/**
* @brief Function implementing the DiagnosticTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDiagnosticTask */
void StartDiagnosticTask(void const * argument)
{
  /* USER CODE BEGIN StartDiagnosticTask */
	static char pcStatsBuffer[256];
  /* Infinite loop */
  for(;;)
  {
    osDelay(6000);

    if(0){
    	  printf("--- Statystyki Czasu Wykonania Zadan ---\r\n");
    	  printf("Zadanie         \tCzas Abs. \t%% Czasu\r\n");
    	  printf("-------------------------------------------\r\n");

          vTaskGetRunTimeStats(pcStatsBuffer);

         char* ptr = pcStatsBuffer;
         while (*ptr != '\0')
         {
            if (*ptr == '\t')
            {
               *ptr = "-";
               __io_putchar(*ptr);
            }
            else
            {
                __io_putchar(*ptr);
            }
            ptr++;
         }
        printf("-------------------------------------------\r\n");
    }

  }
  /* USER CODE END StartDiagnosticTask */
}

/* USER CODE BEGIN Header_StartControl_Task */
/**
* @brief Function implementing the Control_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControl_Task */
void StartControl_Task(void const * argument)
{
  /* USER CODE BEGIN StartControl_Task */

	osEvent event;
    OrientationData_t current_state;
    UART_Data_t Uart_Comunication_Data;
    RC_Commands_t rc_commands;

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	//set_motors_pwm(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
	//osDelay(5000);

    osMessageGet(orientationQueueHandle, osWaitForever);
    osMessageGet(UartDMADataQueueHandle, osWaitForever);

  /* Infinite loop */
  for(;;)
  {
    // --- KROK 1: ZBIERANIE DANYCH ---
    event = osMessageGet(orientationQueueHandle, 100);
    if (event.status == osEventMessage) {
        memcpy(&current_state, (void*)event.value.p, sizeof(OrientationData_t));
    } else {
        set_motors_pwm(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
        continue;
    }
     event = osMessageGet(UartDMADataQueueHandle, 0);
	 if (event.status == osEventMessage) {
	  memcpy(&Uart_Comunication_Data, (void*)event.value.p, sizeof(UART_Data_t));
	}

    read_and_map_rc_commands(&rc_commands, &Uart_Comunication_Data);

    // --- KROK 2: LOGIKA STEROWANIA ---
      if (rc_commands.throttle < THROTTLE_ARM_THRESHOLD) {
          set_motors_pwm(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
          integral_roll_error = 0.0f;
          integral_pitch_error = 0.0f;
          integral_yaw_error = 0.0f;
          continue;
      }
      // --- PID value
      float kp_roll;
      float ki_roll;
      float kd_roll;

	  float kp_pitch;
	  float ki_pitch;
	  float kd_pitch;

	  float kp_yaw;
	  float ki_yaw;
	  float kd_yaw;

      if(1){
    	  kp_roll	=KP_ROLL;
		  ki_roll	=KI_ROLL;
		  kd_roll	=KD_ROLL;

		  kp_pitch	=KP_PITCH;
		  ki_pitch	=KP_PITCH;
		  kd_pitch	=KD_PITCH;

		  kp_yaw	=KP_YAW;
		  ki_yaw	=KI_YAW;
		  kd_yaw	=KD_YAW;
      }else{
    	  kp_roll	=map(Uart_Comunication_Data.vrA, 1000, 2000, 0, 150);
		  ki_roll	=KI_ROLL;
		  kd_roll	=map(Uart_Comunication_Data.vrB, 1000, 2000, 0, 100);

		  kp_pitch	=KP_PITCH;
		  ki_pitch	=KP_PITCH;
		  kd_pitch	=KD_PITCH;

		  kp_yaw	=KP_YAW;
		  ki_yaw	=KI_YAW;
		  kd_yaw	=KD_YAW;
      }


      // --- Obliczenia PID dla każdej osi ---
      // ROLL
      float roll_error = rc_commands.roll_desired_rad - current_state.roll_rad;
      if (fabsf(roll_error) < 0.02f) {
          roll_error = 0.0f;
      }
      integral_roll_error += ki_roll * roll_error * CONTROL_LOOP_PERIOD_S;
      integral_roll_error = constrain(integral_roll_error, -INTEGRAL_LIMIT_RP, INTEGRAL_LIMIT_RP);
      float derivative_roll = -kd_roll * current_state.gx_rps;
      float roll_torque_cmd = kp_roll * roll_error + integral_roll_error + derivative_roll;

      // PITCH
      pitch_error = rc_commands.pitch_desired_rad - current_state.pitch_rad;
      if (fabsf(pitch_error) < 0.02f) {
          pitch_error = 0.0f;
      }

      integral_pitch_error += KI_PITCH * pitch_error * CONTROL_LOOP_PERIOD_S;
      integral_pitch_error = constrain(integral_pitch_error, -INTEGRAL_LIMIT_RP, INTEGRAL_LIMIT_RP);
      float derivative_pitch = -KD_PITCH * current_state.gy_rps;
      float pitch_torque_cmd = KP_PITCH * pitch_error + integral_pitch_error + derivative_pitch;
/*
      // YAW
      // Błąd kąta yaw musi uwzględniać "zawijanie" (np. błąd między -170 a +170 to 20 stopni, a nie 340)
      float yaw_error = rc_commands.yaw_desired_rad - current_state.yaw_rad; // current_state.yaw_rad musi być dostarczone z EKF
      if (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
      if (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

      integral_yaw_error += KI_YAW * yaw_error * CONTROL_LOOP_PERIOD_S;
      integral_yaw_error = constrain(integral_yaw_error, -INTEGRAL_LIMIT_YAW, INTEGRAL_LIMIT_YAW);
      float derivative_yaw = -KD_YAW * current_state.gz_rps;
      float yaw_torque_cmd = KP_YAW * yaw_error + integral_yaw_error + derivative_yaw;
*/

      // --- KROK 3: MIKSER I WYJŚCIE ---
      float base_throttle_pwm = map(rc_commands.throttle, 0.0f, 1.0f, PWM_ARMED_MIN, PWM_MAX);
      /* 		Z YAW
      uint16_t pwm1 = (uint16_t)(base_throttle_pwm + pitch_torque_cmd + roll_torque_cmd - yaw_torque_cmd);
      uint16_t pwm2 = (uint16_t)(base_throttle_pwm - pitch_torque_cmd + roll_torque_cmd + yaw_torque_cmd);
      uint16_t pwm3 = (uint16_t)(base_throttle_pwm - pitch_torque_cmd - roll_torque_cmd - yaw_torque_cmd);
      uint16_t pwm4 = (uint16_t)(base_throttle_pwm + pitch_torque_cmd - roll_torque_cmd + yaw_torque_cmd);
	  */

      uint16_t pwm1 = (uint16_t)(base_throttle_pwm + pitch_torque_cmd + roll_torque_cmd);
      uint16_t pwm2 = (uint16_t)(base_throttle_pwm - pitch_torque_cmd + roll_torque_cmd - 0.0f);
      uint16_t pwm3 = (uint16_t)(base_throttle_pwm - pitch_torque_cmd - roll_torque_cmd - 0.0f);
      uint16_t pwm4 = (uint16_t)(base_throttle_pwm + pitch_torque_cmd - roll_torque_cmd - 0.0f);

      if(Uart_Comunication_Data.swA==1000)
      {
    	  pwm1=PWM_MIN;
    	  pwm2=PWM_MIN;
    	  pwm3=PWM_MIN;
    	  pwm4=PWM_MIN;
      }

      debug_pwm1 = (float)pwm1;
      debug_pwm2 = (float)pwm2;
      debug_pwm3 = (float)pwm3;
      debug_pwm4 = (float)pwm4;

      // Konfiguracja "X"
         // Motor 1 (prawy-przód): +roll, +pitch
         // Motor 2 (prawy-tył): +roll, -pitch
         // Motor 3 (lewy-tył): -roll, -pitch
         // Motor 4 (lewy-przód): -roll, +pitch

      // Ogranicz wartości PWM do bezpiecznego zakresu
      pwm1 = constrain(pwm1, PWM_ARMED_MIN, 900);
      pwm2 = constrain(pwm2, PWM_ARMED_MIN, 900);
      pwm3 = constrain(pwm3, PWM_ARMED_MIN, 900);
      pwm4 = constrain(pwm4, PWM_ARMED_MIN, 900);

      set_motors_pwm(pwm1, pwm2, pwm3, pwm4);

  }
  /* USER CODE END StartControl_Task */
}

/* USER CODE BEGIN Header_StartComunicationTask */
/**
* @brief Function implementing the ComunicationTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartComunicationTask */
void StartComunicationTask(void const * argument)
{
  /* USER CODE BEGIN StartComunicationTask */
	static uint8_t RxData[64];
	static UART_Data_t Uart_Comunication_Data;
	HAL_UART_Receive_DMA(&huart5, RxData, 32);

  /* Infinite loop */
  for(;;)
  {
	      uint32_t notification_value = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

	      if (notification_value > 0) {
	          int start_byte_index = -1;

	          for (int i = 0; i < 31; i++) {
	              if (RxData[i] == 0x20 && RxData[i+1] == 0x40) {
	                  start_byte_index = i;
	                  break;
	              }
	          }

	          if (start_byte_index == -1) {
	              HAL_UART_Receive_DMA(&huart5, RxData, 32);
	              continue;
	          }

	          uint16_t checksum_calculated = 0xFFFF;
	          for (int i = 0; i < 30; i++) {
	              checksum_calculated -= RxData[(start_byte_index + i) % 32];
	          }
	          uint16_t checksum_received = (uint16_t)(RxData[(start_byte_index + 31) % 32] << 8 | RxData[(start_byte_index + 30) % 32]);

	          if (checksum_calculated != checksum_received) {
	              HAL_UART_Receive_DMA(&huart5, RxData, 32);
	              continue;
	          }

	          uint16_t channels[14];
	          for(int i = 0; i < 14; i++) {
	              int lsb_index = (start_byte_index + 2 + i * 2) % 32;
	              int msb_index = (start_byte_index + 3 + i * 2) % 32;
	              channels[i] = (uint16_t)((RxData[msb_index] << 8) | RxData[lsb_index]);
	          }

	          Uart_Comunication_Data.roll     = channels[0];
	          Uart_Comunication_Data.pitch    = channels[1];
	          Uart_Comunication_Data.throttle = channels[2];
	          Uart_Comunication_Data.yaw      = channels[3];
	          Uart_Comunication_Data.swA      = channels[4];
	          Uart_Comunication_Data.swB      = channels[5];
	          Uart_Comunication_Data.swC      = channels[6];
	          Uart_Comunication_Data.swD      = channels[7];
	          Uart_Comunication_Data.vrA      = channels[8];
	          Uart_Comunication_Data.vrB      = channels[9];

	          osMessagePut(UartDMADataQueueHandle, (uint32_t)&Uart_Comunication_Data, 0);

	          HAL_UART_Receive_DMA(&huart5, RxData, 32);

	      } else {
	          printf("comunication task timeout\n");
	          HAL_UART_AbortReceive(&huart5);
	          HAL_UART_Receive_DMA(&huart5, RxData, 32);
	      }
  }
  /* USER CODE END StartComunicationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
