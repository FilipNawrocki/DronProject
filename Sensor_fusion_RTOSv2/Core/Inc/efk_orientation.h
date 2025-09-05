#pragma once

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float phi_rad;
    float theta_rad;

    float P[4];
    float Q[2];
    float R[2];

    float p;
    float q;
    float r;

    // Pola na offsety
    float roll_offset_rad;
    float pitch_offset_rad;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
} KalmanRollPich;

void KalmanRollPich_init(KalmanRollPich *kal, float* gyro_bios, float roll_offset, float pitch_offset, float Pinit, float *Q, float *R);
void KalmanRollPich_Predict(KalmanRollPich *kal, float *gyr_rps, float T);
void KalmanRollPich_Update(KalmanRollPich *kal, float *acc_mps2);




