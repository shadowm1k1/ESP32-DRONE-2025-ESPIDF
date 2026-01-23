#include "../include/mpu.h"
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RAD_TO_DEG 57.2958f  // 180/pi
#define ALPHA 0.98f          // Complementary filter coefficient

static const char *MPUTAG = "MPU6050";

// Gyro Z offset for yaw calibration
static float gyro_z_offset = 0.0f;
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;


// Renamed to avoid conflict
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t mpu_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(10));
}

esp_err_t mpu_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    // Wake up MPU6050
    ESP_ERROR_CHECK(mpu_write_reg(0x6B, 0x00));
    
    ESP_ERROR_CHECK(mpu_write_reg(0x1A, 0x00)); // CONFIG: no DLPF (lowest latency)
    ESP_ERROR_CHECK(mpu_write_reg(0x19, 0x00)); // SMPLRT_DIV = 0 → 8 kHz internal

    // Check WHO_AM_I
    uint8_t who_am_i = 0;
    ESP_ERROR_CHECK(mpu_read_bytes(0x75, &who_am_i, 1));

    if (who_am_i == 0x68) {
        //ESP_LOGI(MPUTAG, "MPU6050 detected (WHO_AM_I=0x%02X)", who_am_i);
        return ESP_OK;
    } else {
        //ESP_LOGE(MPUTAG, "MPU6050 not found! WHO_AM_I=0x%02X", who_am_i);
        return ESP_FAIL;
    }
}

esp_err_t mpu_read_raw(mpu_raw_t *data)
{
    uint8_t buf[14];
    esp_err_t ret = mpu_read_bytes(0x3B, buf, 14);
    if (ret != ESP_OK) {
        //ESP_LOGE(MPUTAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return ret;
    }

    data->accel_x = (buf[0] << 8) | buf[1];
    data->accel_y = (buf[2] << 8) | buf[3];
    data->accel_z = (buf[4] << 8) | buf[5];
    data->gyro_x  = (buf[8] << 8) | buf[9];
    data->gyro_y  = (buf[10] << 8) | buf[11];
    data->gyro_z  = (buf[12] << 8) | buf[13];

    return ESP_OK;
}

/*************** YAW CALIBRATION ****************/
void mpu_calibrate_yaw()
{
    //ESP_LOGI(MPUTAG, "Calibrating yaw... keep the sensor still");

    const int samples = 200;
    int32_t sum = 0;
    mpu_raw_t raw;

    for(int i = 0; i < samples; i++)
    {
        mpu_read_raw(&raw);
        sum += raw.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(5));  // Small delay between samples (~200 × 5ms = 1s)
    }

    gyro_z_offset = (float)sum / samples / 131.0f;
    //ESP_LOGI(MPUTAG, "Yaw gyro offset = %.5f deg/s", gyro_z_offset);
}

void mpu_calibrate_gyro()
{
    const int samples = 200;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
     mpu_raw_t raw;

    for(int i = 0; i < samples; i++)
    {
        mpu_read_raw(&raw);
        sum_z += raw.gyro_z;
        sum_x += raw.gyro_x;
        sum_y += raw.gyro_y;
        vTaskDelay(pdMS_TO_TICKS(5));  // Small delay between samples (~200 × 5ms = 1s)
    }

    gyro_z_offset = (float)sum_z / samples / 131.0f;
    gyro_x_offset = (float)sum_x / samples / 131.0f;
    gyro_y_offset = (float)sum_y / samples / 131.0f;
}
/************************************************/

// Single function to calculate roll/pitch/yaw with complementary filter

mpu_angles_t mpu_get_filtered_angles(mpu_raw_t raw, mpu_angles_t prev, float dt)
{
    mpu_angles_t angles;

    float ax = raw.accel_x / 16384.0f;
    float ay = raw.accel_y / 16384.0f;
    float az = raw.accel_z / 16384.0f;

    float gx = raw.gyro_x / 131.0f;
    float gy = raw.gyro_y / 131.0f;
    float gz = raw.gyro_z / 131.0f;

    float accel_roll  = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
    float accel_pitch = -atan2f(ay, az) * RAD_TO_DEG;

    angles.roll  = ALPHA * (prev.roll  + gx * dt) + (1 - ALPHA) * accel_roll;
    angles.pitch = ALPHA * (prev.pitch + gy * dt) + (1 - ALPHA) * accel_pitch;

    // Yaw = integrate gyro Z minus calibrated offset
    float gz_corrected = gz - gyro_z_offset;
    angles.yaw = prev.yaw + gz_corrected * dt;

    return angles;
}

mpu_rates_t mpu_get_rates(mpu_raw_t raw)
{
    mpu_rates_t rates;

    rates.rate_roll = (raw.gyro_y / 131.0f) - gyro_y_offset;
    rates.rate_pitch = (raw.gyro_x / 131.0f) - gyro_x_offset;
    rates.rate_yaw = (raw.gyro_z / 131.0f) - gyro_z_offset;
    
    //ESP_LOGI(MPUTAG, "%f   %f",rates.rate_pitch ,rates.rate_roll);

    return rates;
}

void mpuDebugPrint(mpu_raw_t raw_data)
{
   // ESP_LOGI(MPUTAG, "Accel [X:%6d  Y:%6d  Z:%6d] | Gyro [X:%6d  Y:%6d  Z:%6d]", raw_data.accel_x, raw_data.accel_y, raw_data.accel_z,raw_data.gyro_x, raw_data.gyro_y, raw_data.gyro_z);
}

void mpuPrintAngles(mpu_angles_t ang)
{
   // ESP_LOGI(MPUTAG, "Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f",ang.roll, ang.pitch, ang.yaw);
}
