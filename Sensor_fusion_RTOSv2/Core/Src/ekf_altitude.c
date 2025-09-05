#include "ekf_altitude.h"
#include <math.h>

const float Q_ACCEL = 0.5f;
const float Q_BIAS = 0.001f;
const float R_BARO = 1.0f;

void force_symmetry(float P[9]) {
    // P[1,0] = P[0,1]
    P[3] = P[1];
    // P[2,0] = P[0,2]
    P[6] = P[2];
    // P[2,1] = P[1,2]
    P[7] = P[5];
}

void EKF_Altitude_Init(EKF_Altitude_t* ekf, float initial_altitude) {
    ekf->x[0] = -initial_altitude; // p_z
    ekf->x[1] = 0.0f;             // v_z
    ekf->x[2] = 0.0f;             // b_az

    // Inicjalizacja macierzy kowariancji P
    for (int i = 0; i < STATE_SIZE_ALT * STATE_SIZE_ALT; i++) {
        ekf->P[i] = 0.0f;
    }
    ekf->P[0] = R_BARO;  // Początkowa niepewność pozycji jest równa niepewności pomiaru
    ekf->P[4] = 1.0f;    // Niepewność prędkości
    ekf->P[8] = 0.1f;    // Niepewność biasu
}

void EKF_Altitude_Predict(EKF_Altitude_t* ekf, float acc_z_corrected, float dt) {
    // Odczyt aktualnego stanu
    float pos = ekf->x[0];
    float vel = ekf->x[1];
    float bias = ekf->x[2];

    float acc_true = acc_z_corrected - bias;

    // --- Krok 1: Predykcja Stanu ---
    ekf->x[0] = pos + vel * dt + 0.5f * acc_true * dt * dt;
    ekf->x[1] = vel + acc_true * dt;


    // --- Krok 2: Predykcja Kowariancji P' = A*P*A^T + Q ---
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    // Zapisz starą macierz P
    float P_old[9];
    for(int i=0; i<9; i++) P_old[i] = ekf->P[i];

    // Dla czytelności, przypisz elementy P_old do zmiennych
    float p00=P_old[0], p01=P_old[1], p02=P_old[2];
    float p11=P_old[4], p12=P_old[5];
    float p22=P_old[8];


    // Krok pośredni: Temp = A * P_old
    float T00 = p00 + dt * p01 - 0.5f * dt2 * p02;
    float T01 = p01 + dt * p11 - 0.5f * dt2 * p12;
    float T02 = p02 + dt * p12 - 0.5f * dt2 * p22;
    float T11 = p11 - dt * p12;
    float T12 = p12 - dt * p22;


    // Krok końcowy: P_new = Temp * A_T
    ekf->P[0] = T00 + T01 * dt - T02 * 0.5f * dt2;
    ekf->P[1] = T01 - T02 * dt;
    ekf->P[2] = T02;
    ekf->P[4] = T11 - T12 * dt;
    ekf->P[5] = p12 - dt * p22; // Można podstawić T12
    ekf->P[8] = p22; // p_new_22 = T22 = p22

        // Uzupełnij dolny trójkąt przez symetrię
    ekf->P[3] = ekf->P[1];
    ekf->P[6] = ekf->P[2];
    ekf->P[7] = ekf->P[5];

        // Dodaj macierz szumu procesowego Q

    ekf->P[0] += 0.25f * dt4 * Q_ACCEL;
    ekf->P[1] += 0.5f * dt3 * Q_ACCEL;
    ekf->P[3] += 0.5f * dt3 * Q_ACCEL;
    ekf->P[4] += dt2 * Q_ACCEL;
    ekf->P[8] += Q_BIAS * dt;

    force_symmetry(ekf->P);
}

void EKF_Altitude_Update(EKF_Altitude_t* ekf, float baro_altitude) {
    if (isnan(baro_altitude)) return;

    const float H[3] = {-1.0f, 0.0f, 0.0f};

    // Innowacja (błąd)
    float predicted_altitude = -ekf->x[0];
    float innovation = baro_altitude - predicted_altitude;

    // Kowariancja innowacji
    float S = ekf->P[0] + R_BARO;
    if (S < 1e-9f) return;

    // Oblicz kwadrat błędu Mahalanobisa
    // d^2 = innovation * S_inv * innovation
    float d_squared = (innovation * innovation) / S;

    // Próg odrzucenia - np. dla 3 sigma.
    // Dla 1 wymiaru, 3-sigma odpowiada wartości chi-kwadrat ~9.0
    const float GATE_THRESHOLD = 9.0f;
    if (d_squared > GATE_THRESHOLD) {
        return;
    }

    float S_inv = 1.0f / S;

    // Wzmocnienie Kalmana
    float K[3];
    K[0] = ekf->P[0] * H[0] * S_inv;
    K[1] = ekf->P[3] * H[0] * S_inv; // użyliśmy P[3] bo P jest symetryczna P[1,0] == P[0,1]
    K[2] = ekf->P[6] * H[0] * S_inv; // użyliśmy P[6] bo P[2,0] == P[0,2]

    // Aktualizacja stanu
    ekf->x[0] += K[0] * innovation;
    ekf->x[1] += K[1] * innovation;
    ekf->x[2] += K[2] * innovation;

    // --- Aktualizacja Kowariancji - NIEPEŁNA FORMA JOSEPHA ---
    // P_new = (I - K*H) * P_old

    float P_old[9];
    for(int i=0; i<9; i++) P_old[i] = ekf->P[i];

    // Macierz (I - K*H)
    float I_m_KH[3][3] = {
        {1.0f - K[0] * H[0], -K[0] * H[1], -K[0] * H[2]},
        {-K[1] * H[0], 1.0f - K[1] * H[1], -K[1] * H[2]},
        {-K[2] * H[0], -K[2] * H[1], 1.0f - K[2] * H[2]}
    };

    // Ponieważ H = [-1, 0, 0], upraszcza się do:
    // I_m_KH = { {1.0f + K[0], 0, 0},
    //            {K[1],        1, 0},
    //            {K[2],        0, 1} };

    // P_new = (I - KH) * P_old
    ekf->P[0] = I_m_KH[0][0] * P_old[0]; // (1+K0)*p00
    ekf->P[1] = I_m_KH[0][0] * P_old[1]; // (1+K0)*p01
    ekf->P[2] = I_m_KH[0][0] * P_old[2]; // (1+K0)*p02

    ekf->P[3] = I_m_KH[1][0] * P_old[0] + P_old[3]; // K1*p00 + p10
    ekf->P[4] = I_m_KH[1][0] * P_old[1] + P_old[4]; // K1*p01 + p11
    ekf->P[5] = I_m_KH[1][0] * P_old[2] + P_old[5]; // K1*p02 + p12

    ekf->P[6] = I_m_KH[2][0] * P_old[0] + P_old[6]; // K2*p00 + p20
    ekf->P[7] = I_m_KH[2][0] * P_old[1] + P_old[7]; // K2*p01 + p21
    ekf->P[8] = I_m_KH[2][0] * P_old[2] + P_old[8]; // K2*p02 + p22

    force_symmetry(ekf->P);
}


