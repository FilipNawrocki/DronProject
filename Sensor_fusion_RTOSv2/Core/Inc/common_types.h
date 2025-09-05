#pragma once

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t pressure_raw;
    uint32_t timestamp_mpu;
    uint8_t baro_data_updated;
} RawSensorData_t;


typedef struct {
    float pitch_rad;
    float roll_rad;
    //float yaw_rate_rps;
    float altitude;

    float gx_rps; // Prędkość kątowa p [rad/s]
    float gy_rps; // Prędkość kątowa q [rad/s]
    float gz_rps; // Prędkość kątowa r [rad/s]
} OrientationData_t;

typedef struct {
	float throttle;
	float roll_desired_rad;
	float pitch_desired_rad;
	float yaw_rate_desired_rps;
} RC_Commands_t;

typedef struct {
	uint16_t throttle;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t swA;
	uint16_t swB;
	uint16_t swC;
	uint16_t swD;
	uint16_t vrA;
	uint16_t vrB;
}UART_Data_t;
