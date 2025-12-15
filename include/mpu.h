#pragma once

#include "esp_err.h"
#include "driver/i2c.h"

#define MPU6050_ADDR         0x68
#define I2C_MASTER_SDA_IO    11
#define I2C_MASTER_SCL_IO    10
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   400000

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu_raw_t;

// Struct for filtered roll and pitch angles (Angle Mode)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} mpu_angles_t;

// NEW: Struct for gyro rates (Rate Mode)
typedef struct {
    float rate_roll;   // deg/s
    float rate_pitch;  // deg/s
    float rate_yaw;    // deg/s
} mpu_rates_t;

// MPU6050 initialization and data read
esp_err_t mpu_init(void);
esp_err_t mpu_read_raw(mpu_raw_t *data);

// NEW: Rate Mode functions
void mpu_calibrate_gyro(void);
mpu_rates_t mpu_get_rates(mpu_raw_t raw);
void mpuPrintRates(mpu_rates_t rates);

// OLD: Angle Mode functions (optional, keep for future)
void mpu_calibrate_yaw(void);
mpu_angles_t mpu_get_filtered_angles(mpu_raw_t raw, mpu_angles_t prev_angles, float dt);
void mpuPrintAngles(mpu_angles_t angles);
void mpuDebugPrint(mpu_raw_t rawdata);
