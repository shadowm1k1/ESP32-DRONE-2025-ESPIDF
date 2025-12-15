#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"  
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

float m0, m1, m2, m3;
volatile float baseThrottle = 0;
volatile bool killswitch = true;

// Setpoints: desired rotation speed in deg/s
volatile float setpoint_roll = 0.0f;
volatile float setpoint_pitch = 0.0f;
volatile float setpoint_yaw = 0.0f;

// PID tuning (adjustable via WiFi)
volatile float rollp = 0.3f, rolli = 0.05f, rolld = 0.01f;
volatile float pitchp = 0.3f, pitchi = 0.05f, pitchd = 0.01f;
volatile float yawp = 0.5f, yawi = 0.02f, yawd = 0.0f;

PID_t pid_roll, pid_pitch, pid_yaw;

void control_task(void *pvParameters)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    TickType_t last_wake_time = xTaskGetTickCount();

    int64_t last_time = esp_timer_get_time();
    static int sensor_error_count = 0;

    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) * 1e-6f;
        last_time = now;

        if (dt < 0.0005f) dt = 0.0005f;
        if (dt > 0.0050f) dt = 0.0050f;

        // Read raw sensor data
        mpu_raw_t raw_data;
        mpu_rates_t rates = {0};
        
        if (mpu_read_raw(&raw_data) == ESP_OK) {
            sensor_error_count = 0;
            rates = mpu_get_rates(raw_data);  // Only gyro, no accel!
        }
        else {
            sensor_error_count++;
            if (sensor_error_count > 10) {
                killswitch = true;
            }
        }
        
        // Update PID parameters
        PID_SetTunings(&pid_roll, rollp, rolli, rolld);
        PID_SetTunings(&pid_pitch, pitchp, pitchi, pitchd);
        PID_SetTunings(&pid_yaw, yawp, yawi, yawd);
        
        // PID: setpoint (desired rate) - measured rate (from gyro)
        float roll_output  = PID_Compute(&pid_roll,  setpoint_roll,  rates.rate_roll,  dt); 
        float pitch_output = PID_Compute(&pid_pitch, setpoint_pitch, rates.rate_pitch, dt);
        float yaw_output   = PID_Compute(&pid_yaw,   setpoint_yaw,   rates.rate_yaw,   dt);

        // Motor mixing
        m0 = baseThrottle + roll_output + pitch_output + yaw_output;
        m1 = baseThrottle - roll_output + pitch_output - yaw_output;
        m2 = baseThrottle - roll_output - pitch_output + yaw_output;
        m3 = baseThrottle + roll_output - pitch_output - yaw_output;

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

            pid_roll.integral = 0;
            pid_pitch.integral = 0;
            pid_yaw.integral = 0;
        }
        
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

void app_main(void)
{
    wifi_connection();
    Motor_Init();

    if (mpu_init() != ESP_OK) {
        return;
    }
    
    mpu_calibrate_gyro();  // Calibrate gyro offsets
    
    PID_Init(&pid_roll,  0.3f, 0.05f, 0.01f);
    PID_Init(&pid_pitch, 0.3f, 0.05f, 0.01f);
    PID_Init(&pid_yaw,   0.5f, 0.02f, 0.0f);

    PID_SetOutputLimits(&pid_roll,  -300, 300);
    PID_SetOutputLimits(&pid_pitch, -300, 300);
    PID_SetOutputLimits(&pid_yaw,   -150, 150);
    
    xTaskCreatePinnedToCore(control_task, "control_task", 8192, NULL, 9, NULL, 0);

    vTaskDelete(NULL);
}
