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

// Struct for filtered roll and pitch angles
typedef struct {
    float roll;
    float pitch;
    float yaw;
} mpu_angles_t;

typedef struct 
{
    float rate_roll;
    float rate_pitch;
    float rate_yaw;
}mpu_rates_t;


// MPU6050 initialization and data read
esp_err_t mpu_init(void);
void mpu_calibrate_yaw();
esp_err_t mpu_read_raw(mpu_raw_t *data);
void mpu_calibrate_gyro(void);
mpu_rates_t mpu_get_rates(mpu_raw_t raw);

// Debug printing
void mpuDebugPrint(mpu_raw_t rawdata);

// Calculate roll/pitch with complementary filter
mpu_angles_t mpu_get_filtered_angles(mpu_raw_t raw, mpu_angles_t prev_angles, float dt);

// Print filtered angles
void mpuPrintAngles(mpu_angles_t angles);
