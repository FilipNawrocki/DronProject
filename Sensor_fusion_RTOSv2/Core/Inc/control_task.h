

#ifndef INC_CONTROL_TASK_H_
#define INC_CONTROL_TASK_H_

#include "control_task.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "math.h"
#include "common_types.h"
#include "tim.h"

extern osMessageQId orientationQueueHandle;
extern TIM_HandleTypeDef htim3;


#define CONTROL_LOOP_PERIOD_MS 10
#define CONTROL_LOOP_PERIOD_S  (CONTROL_LOOP_PERIOD_MS / 1000.0f)

#define KP_ROLL  80.0f
#define KI_ROLL  2.0f
#define KD_ROLL  30.0f

#define KP_PITCH 80.0f
#define KI_PITCH 2.0f
#define KD_PITCH 30.0f

#define KP_YAW   0.0f
#define KI_YAW   0.0f
#define KD_YAW   0.0f

// Limity
#define MAX_ROLL_PITCH_CMD_RAD   (35.0f * M_PI / 180.0f)
#define MAX_YAW_CMD_RAD          (180.0f * M_PI / 180.0f) // Pełen zakres dla yaw
#define INTEGRAL_LIMIT_RP        50.0f  // Limit dla integratora Roll/Pitch
#define INTEGRAL_LIMIT_YAW       50.0f  // Limit dla integratora Yaw

#define PWM_MIN 500
#define PWM_MAX 1000
#define PWM_ARMED_MIN 500
#define THROTTLE_ARM_THRESHOLD 0.05f

static float integral_roll_error = 0.0f;
static float integral_pitch_error = 0.0f;
static float integral_yaw_error = 0.0f;


float map(float value, float in_min, float in_max, float out_min, float out_max);
float constrain(float value, float min, float max);
void read_and_map_rc_commands(RC_Commands_t* commands, UART_Data_t* raw_Data );
void set_motors_pwm(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4);


#endif /* INC_CONTROL_TASK_H_ */
