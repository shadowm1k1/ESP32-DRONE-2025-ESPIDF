#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"


#include "../include/motor.h"
#include "../include/mpu.h"
#include "../include/pid.h"
#include "../include/wifi.h"



#define BASE_THROTTLE 100


mpu_angles_t angles = {0}; // Initialize filtered angles
float m0, m1, m2, m3;
bool killswitch = true;
float v1,v2,v3,v4;

void app_main(void)
{
    wifi_connection();
    // Initialize motors
    Motor_Init();

    // Initialize MPU6050
    if (mpu_init() != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize MPU6050");
        return;
    }

    mpu_raw_t raw_data;
    

    int64_t last_time = esp_timer_get_time(); // microseconds

    
    PID_t pid_roll, pid_pitch;
    PID_Init(&pid_roll, 1.0f, 0.06f, 0.2f); //konstanten
    PID_Init(&pid_pitch, 1.0f, 0.06f, 0.2f); //kostanten

    PID_SetOutputLimits(&pid_roll, -30, 30);
    PID_SetOutputLimits(&pid_pitch, -30, 30);
    

    while (1) {
        int64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f; // convert us to seconds
        last_time = current_time;

        // Read raw accelerometer and gyroscope data
        if (mpu_read_raw(&raw_data) == ESP_OK) {
            // Calculate filtered roll/pitch
            angles = mpu_get_filtered_angles(raw_data, angles, dt);

            float roll_output = PID_Compute(&pid_roll, 0.0f, angles.roll, dt);
            float pitch_output = PID_Compute(&pid_pitch, 0.0f, angles.pitch, dt);

            m0 = BASE_THROTTLE - roll_output + pitch_output; //top left
            m1 = BASE_THROTTLE - roll_output - pitch_output; //top right
            m2 = BASE_THROTTLE + roll_output - pitch_output; //bot right
            m3 = BASE_THROTTLE + roll_output + pitch_output; //bot left
            

            // Print filtered angles
            //mpuPrintAngles(angles);

            if (!killswitch) //hier sollte eig killswitch sein von controller
            {
                Motor_SetDuty(m0, 0);
                Motor_SetDuty(m1, 1);
                Motor_SetDuty(m2, 2);
                Motor_SetDuty(m3, 3);
            }
            else
            {
                Motor_SetDuty(0, 0);
                Motor_SetDuty(0, 1);
                Motor_SetDuty(0, 2);
                Motor_SetDuty(0, 3);
            }
                
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
