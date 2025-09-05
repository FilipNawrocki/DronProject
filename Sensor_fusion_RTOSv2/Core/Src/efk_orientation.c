#include <efk_orientation.h>



void KalmanRollPich_init(KalmanRollPich *kal, float* gyro_bias, float roll_offset, float pitch_offset, float Pinit, float *Q, float *R) {
    kal->phi_rad = 0.0f;
    kal->theta_rad = 0.0f;
    kal->p = 0.0f;
    kal->q = 0.0f;
    kal->r = 0.0f;

    kal->roll_offset_rad = roll_offset;
    kal->pitch_offset_rad = pitch_offset;

    kal->gyro_bias_x = gyro_bias[0];
    kal->gyro_bias_y = gyro_bias[1];
    kal->gyro_bias_z = gyro_bias[2];

	kal->P[0] = Pinit;	kal ->P[1] = 0.0f;
	kal->P[2] = 0.0f;	kal ->P[3] = Pinit;

	kal->Q[0] = Q[0];	kal ->Q[1] = Q[1];
	kal->R[0] = R[0];	kal->R[1] = R[1];;
}
void KalmanRollPich_Predict(KalmanRollPich *kal, float *gyr_rps, float T){

	kal->p = gyr_rps[0] - kal->gyro_bias_x;
	kal->q = gyr_rps[1] - kal->gyro_bias_y;
	kal->r = gyr_rps[2] - kal->gyro_bias_z;

	/*	predict	*/

	/* common trig term*/
	float sp = sin(kal->phi_rad); float cp = cos(kal->phi_rad); float tt = tan(kal->theta_rad);

	/* x+ = x- + T * f(x,u) */
	kal->phi_rad = kal->phi_rad + T * (kal->p + tt*(kal->q * sp + kal->r * cp));
	kal->theta_rad = kal->theta_rad + T*( kal->p * cp - kal->r * sp);

	/*recompute common trig*/
	sp = sin(kal->phi_rad); cp = cos(kal->phi_rad);
	float st = sin(kal->theta_rad); float ct = cos(kal->theta_rad);
	tt = st/ct;

	/*Jacobian of f(x,u)*/
	float A[4] = { tt * (kal->q * cp - kal->r * sp), (kal->r * cp + kal->q * sp)*(tt * tt + 1.0f),
			-(kal->r * cp + kal->q * sp),	 0.0f};

	/*update cov matrix P+ = P- + T*(A*P- + P-*A' + Q)*/


	// Compute A*P
	float AP[4];
	AP[0] = A[0] * kal->P[0] + A[1] * kal->P[2];
	AP[1] = A[0] * kal->P[1] + A[1] * kal->P[3];
	AP[2] = A[2] * kal->P[0] + A[3] * kal->P[2];
	AP[3] = A[2] * kal->P[1] + A[3] * kal->P[3];

	// Compute P*A' (A' to transpos A)
	float PAT[4];
	PAT[0] = kal->P[0] * A[0] + kal->P[1] * A[1];
	PAT[1] = kal->P[0] * A[2] + kal->P[1] * A[3];
	PAT[2] = kal->P[2] * A[0] + kal->P[3] * A[1];
	PAT[3] = kal->P[2] * A[2] + kal->P[3] * A[3];

	// Compute P_dot = A*P + P*A' + Q
	float P_dot[4];
	P_dot[0] = AP[0] + PAT[0] + kal->Q[0];
	P_dot[1] = AP[1] + PAT[1];
	P_dot[2] = AP[2] + PAT[2];
	P_dot[3] = AP[3] + PAT[3] + kal->Q[1];

	// Aktualizuj P (metoda Eulera)
	kal->P[0] += P_dot[0] * T;
	kal->P[1] += P_dot[1] * T;
	kal->P[2] += P_dot[2] * T;
	kal->P[3] += P_dot[3] * T;


}
void KalmanRollPich_Update(KalmanRollPich *kal, float *acc_mps2){
	    float ax = acc_mps2[0];
	    float ay = acc_mps2[1];
	    float az = acc_mps2[2];

	    // Obliczenie "pomiaru" z akcelerometru
	    float phi_measured_raw   = atan2f(ay, az);
	    float theta_measured_raw = atan2f(-ax, sqrtf(ay * ay + az * az));

	    // --- POPRAWKA: Odejmij offsety od pomiaru ---
	    float phi_measured   = phi_measured_raw - kal->roll_offset_rad;
	    float theta_measured = theta_measured_raw - kal->pitch_offset_rad;

    // --- Krok 3: Obliczenie wzmocnienia Kalmana (Kalman Gain) K ---
    // K = P * H' * (H * P * H' + R)^-1
    // Ponieważ H = I, wzór upraszcza się do: K = P * (P + R)^-1

    // Oblicz mianownik (P + R)
    float S[4]; // Macierz innowacji kowariancji
    S[0] = kal->P[0] + kal->R[0]; // S(0,0) = P(0,0) + R_phi
    S[1] = kal->P[1];             // S(0,1) = P(0,1)
    S[2] = kal->P[2];             // S(1,0) = P(1,0)
    S[3] = kal->P[3] + kal->R[1]; // S(1,1) = P(1,1) + R_theta

    // Oblicz odwrotność macierzy S (dla macierzy 2x2)
    // S_inv = 1/det(S) * | S(1,1) -S(0,1) |
    //                    | -S(1,0)  S(0,0) |
    float det_S = S[0] * S[3] - S[1] * S[2];
    if (fabsf(det_S) < 1e-9f) {
        // Unikamy dzielenia przez zero, jeśli macierz jest osobliwa
        return;
    }
    float S_inv[4];
    S_inv[0] =  S[3] / det_S;
    S_inv[1] = -S[1] / det_S;
    S_inv[2] = -S[2] / det_S;
    S_inv[3] =  S[0] / det_S;

    // Oblicz wzmocnienie Kalmana K = P * S_inv (ponieważ H' = I)
    float K[4]; // Kalman Gain
    K[0] = kal->P[0] * S_inv[0] + kal->P[1] * S_inv[2];
    K[1] = kal->P[0] * S_inv[1] + kal->P[1] * S_inv[3];
    K[2] = kal->P[2] * S_inv[0] + kal->P[3] * S_inv[2];
    K[3] = kal->P[2] * S_inv[1] + kal->P[3] * S_inv[3];

    // --- Krok 4: Korekta estymaty stanu x+ = x- + K * (z - h(x-)) ---

    // Oblicz błąd pomiaru (innowację) y = z - h(x)
    // h(x) to nasz przewidziany stan [phi_rad, theta_rad]
    float y_phi   = phi_measured - kal->phi_rad;
    float y_theta = theta_measured - kal->theta_rad;

    // Skoryguj stan
    kal->phi_rad   += K[0] * y_phi + K[1] * y_theta;
    kal->theta_rad += K[2] * y_phi + K[3] * y_theta;

    // --- Krok 5: Korekta macierzy kowariancji błędu P+ = (I - K*H) * P- ---
    // Ponieważ H = I, wzór to: P+ = (I - K) * P-

    // Oblicz (I - K)
    float I_minus_K[4];
    I_minus_K[0] = 1.0f - K[0];
    I_minus_K[1] = -K[1];
    I_minus_K[2] = -K[2];
    I_minus_K[3] = 1.0f - K[3];

    // Zapisz starą macierz P, bo będzie potrzebna w obliczeniach
    float P_old[4] = {kal->P[0], kal->P[1], kal->P[2], kal->P[3]};

    // Oblicz nową macierz P+ = (I - K) * P_old
    kal->P[0] = I_minus_K[0] * P_old[0] + I_minus_K[1] * P_old[2];
    kal->P[1] = I_minus_K[0] * P_old[1] + I_minus_K[1] * P_old[3];
    kal->P[2] = I_minus_K[2] * P_old[0] + I_minus_K[3] * P_old[2];
    kal->P[3] = I_minus_K[2] * P_old[1] + I_minus_K[3] * P_old[3];


}


