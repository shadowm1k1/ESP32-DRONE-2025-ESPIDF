#include "../include/mpu.h"
#include "sdkconfig.h"  
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *MPUTAG = "MPU6050";

// Gyro offsets for calibration
static float gyro_x_offset = 0.0f;  
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t mpu_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
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

    // Check WHO_AM_I
    uint8_t who_am_i = 0;
    ESP_ERROR_CHECK(mpu_read_bytes(0x75, &who_am_i, 1));

    if (who_am_i == 0x68) {
        ESP_LOGI(MPUTAG, "MPU6050 detected");
        return ESP_OK;
    } else {
        ESP_LOGE(MPUTAG, "MPU6050 not found! WHO_AM_I=0x%02X", who_am_i);
        return ESP_FAIL;
    }
}

esp_err_t mpu_read_raw(mpu_raw_t *data)
{
    uint8_t buf[14];
    esp_err_t ret = mpu_read_bytes(0x3B, buf, 14);
    if (ret != ESP_OK) {
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

/*************** GYRO CALIBRATION ****************/
void mpu_calibrate_gyro()
{
    ESP_LOGI(MPUTAG, "Calibrating gyro... keep still!");

    const int samples = 200;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    mpu_raw_t raw;

    for(int i = 0; i < samples; i++)
    {
        mpu_read_raw(&raw);
        sum_x += raw.gyro_x;
        sum_y += raw.gyro_y;
        sum_z += raw.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gyro_x_offset = (float)sum_x / samples / 131.0f;
    gyro_y_offset = (float)sum_y / samples / 131.0f;
    gyro_z_offset = (float)sum_z / samples / 131.0f;
    
    ESP_LOGI(MPUTAG, "Offsets: X=%.2f Y=%.2f Z=%.2f deg/s", 
             gyro_x_offset, gyro_y_offset, gyro_z_offset);
}

/*************** RATE MODE: Pure Gyro ****************/
mpu_rates_t mpu_get_rates(mpu_raw_t raw)
{
    mpu_rates_t rates;
    
    // ONLY gyro data, no accelerometer!
    rates.rate_roll  = (raw.gyro_x / 131.0f) - gyro_x_offset;
    rates.rate_pitch = (raw.gyro_y / 131.0f) - gyro_y_offset;
    rates.rate_yaw   = (raw.gyro_z / 131.0f) - gyro_z_offset;
    
    return rates;
}

void mpuPrintRates(mpu_rates_t rates)
{
    ESP_LOGI(MPUTAG, "Roll: %6.1f | Pitch: %6.1f | Yaw: %6.1f deg/s",
             rates.rate_roll, rates.rate_pitch, rates.rate_yaw);
}
