#include "control_task.h"

float map(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void read_and_map_rc_commands(RC_Commands_t* commands, UART_Data_t* raw_Data) {

    commands->throttle = map(raw_Data->throttle, 1000.0f, 2000.0f, 0.0f, 1.0f);
    commands->roll_desired_rad = map(raw_Data->roll, 1000.0f, 2000.0f, -MAX_ROLL_PITCH_CMD_RAD, MAX_ROLL_PITCH_CMD_RAD);
    commands->pitch_desired_rad = map(raw_Data->pitch, 1000.0f, 2000.0f, -MAX_ROLL_PITCH_CMD_RAD, MAX_ROLL_PITCH_CMD_RAD);
    //commands->yaw_desired_rad = map(raw_Data->yaw, 1000.0f, 2000.0f, -MAX_YAW_CMD_RAD, MAX_YAW_CMD_RAD);
}

// Funkcja ustawiająca PWM dla wszystkich 4 silników
void set_motors_pwm(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm1); // Motor 1 (prawy-przód, CW)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm2); // Motor 2 (prawy-tył, CCW)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm3); // Motor 3 (lewy-tył, CW)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm4); // Motor 4 (lewy-przód, CCW)
}
