#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../include/motor.h"
#include "../include/mpu.h"
#include "esp_log.h"
#include "esp_timer.h"

void app_main(void)
{
    // Initialize motors
    Motor_Init();

    // Initialize MPU6050
    if (mpu_init() != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize MPU6050");
        return;
    }

    mpu_raw_t raw_data;
    mpu_angles_t angles = {0}; // Initialize filtered angles

    int64_t last_time = esp_timer_get_time(); // microseconds

    while (1) {
        int64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f; // convert us to seconds
        
        last_time = current_time;

        // Read raw accelerometer and gyroscope data
        if (mpu_read_raw(&raw_data) == ESP_OK) {
            // Calculate filtered roll/pitch
            angles = mpu_get_filtered_angles(raw_data, angles, dt);

            // Print filtered angles
            mpuPrintAngles(angles);
        }

    }
}
