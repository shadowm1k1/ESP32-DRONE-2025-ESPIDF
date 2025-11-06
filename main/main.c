#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <math.h>



#include "../include/motor.h"
#include "../include/mpu.h"
#include "../include/pid.h"
#include "../include/wifi.h"



#define CONTROL_HZ           100
#define CONTROL_PERIOD_US    (1000000 / CONTROL_HZ) 


mpu_angles_t angles = {0}; // Initialize filtered angles
float m0, m1, m2, m3;
volatile float baseThrottle = 0;
volatile bool killswitch = true;
volatile float contthrottle, controll, contpitch, contyaw;
volatile float rollp,rolli,rolld,pitchp,pitchi,pitchd,yawp,yawi,yawd;

PID_t pid_roll, pid_pitch, pid_yaw;


void control_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
    int64_t last_time = esp_timer_get_time();    // microseconds for dt

    while (1) {

        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) / 1000000.0f; // convert us -> seconds
        last_time = now;

        // --- Sensor read + PID + motor update ---
        mpu_raw_t raw_data;
        if (mpu_read_raw(&raw_data) == ESP_OK) {
            angles = mpu_get_filtered_angles(raw_data, angles, dt);
        }
        
        float roll_output  = PID_Compute(&pid_roll,  0.0f, angles.roll, dt);
        float pitch_output = PID_Compute(&pid_pitch, 0.0f, angles.pitch, dt);
        float yaw_output   = PID_Compute(&pid_yaw,   0.0f, angles.yaw,  dt);

        m0 = baseThrottle + roll_output + pitch_output + yaw_output;
        m1 = baseThrottle + roll_output - pitch_output - yaw_output;
        m2 = baseThrottle - roll_output - pitch_output + yaw_output;
        m3 = baseThrottle - roll_output + pitch_output - yaw_output;

        m0 = fminf(fmaxf(m0, 0.0f), 1023.0f);
        m1 = fminf(fmaxf(m1, 0.0f), 1023.0f);
        m2 = fminf(fmaxf(m2, 0.0f), 1023.0f);
        m3 = fminf(fmaxf(m3, 0.0f), 1023.0f);

        if (!killswitch) {
            Motor_SetDuty((uint16_t)m0, 0);
            Motor_SetDuty((uint16_t)m1, 1);
            Motor_SetDuty((uint16_t)m2, 2);
            Motor_SetDuty((uint16_t)m3, 3);
        } else {
            Motor_SetDuty(0, 0);
            Motor_SetDuty(0, 1);
            Motor_SetDuty(0, 2);
            Motor_SetDuty(0, 3);
        }
        vTaskDelayUntil(&last_wake_time, period);
    }
}


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
    mpu_calibrate_yaw();
    
    PID_Init(&pid_roll, 0.5f, 0.0f, 0.0f); //konstanten
    PID_Init(&pid_pitch, 0.5f, 0.00f, 0.0f); //kostanten
    PID_Init(&pid_yaw, 0.0f, 0.00f, 0.0f); //kostanten

    PID_SetOutputLimits(&pid_roll, -40, 40);
    PID_SetOutputLimits(&pid_pitch, -40, 40);
    PID_SetOutputLimits(&pid_yaw, -40, 40);

    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 9, NULL, 0);

    // optionally keep app_main busy or delete task:
    vTaskDelete(NULL);
    /*
    PID_SetTunings(&pid_roll,rollp,rolli,rolld);
    PID_SetTunings(&pid_pitch,pitchp,pitchi,pitchd);
    PID_SetTunings(&pid_yaw,yawp,yawi,yawd);
    */
    
}
