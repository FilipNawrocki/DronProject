#pragma once
#define STATE_SIZE_ALT 3

typedef struct {
    // Wektor stanu [pozycja_z, prędkość_z, bias_akcelerometru_z]
    float x[STATE_SIZE_ALT];

    // Macierz kowariancji błędu estymacji (3x3)
    float P[STATE_SIZE_ALT * STATE_SIZE_ALT];

} EKF_Altitude_t;

/**
 * @brief Inicjalizuje filtr EKF do estymacji wysokości.
 */
void EKF_Altitude_Init(EKF_Altitude_t* ekf, float initial_altitude);

/**
 * @brief Krok predykcji filtru, używający danych z akcelerometru.
 */
void EKF_Altitude_Predict(EKF_Altitude_t* ekf, float acc_z_world, float dt);

/**
 * @brief Krok korekty filtru, używający danych z barometru.
 */
void EKF_Altitude_Update(EKF_Altitude_t* ekf, float baro_altitude);


