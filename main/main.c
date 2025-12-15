#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <math.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"



#include "../include/motor.h"
#include "../include/mpu.h"
#include "../include/pid.h"
#include "../include/wifi.h"



mpu_angles_t angles = {0}; // Initialize filtered angles
float m0, m1, m2, m3;
volatile float baseThrottle = 0;
volatile bool killswitch = true;
volatile float contthrottle, controll, contpitch, contyaw;
volatile float rollp,rolli,rolld,pitchp,pitchi,pitchd,yawp,yawi,yawd;

volatile float set_rate_roll = 0.0f; //deg/s
volatile float set_rate_pitch = 0.0f;
volatile float set_rate_yaw = 0.0f;

PID_t pid_roll, pid_pitch, pid_yaw;


void control_task(void *pvParameters)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    TickType_t last_wake_time = xTaskGetTickCount();

    int64_t last_time = esp_timer_get_time();
    static int sensor_error_count = 0;

    
    int64_t last_print_time = esp_timer_get_time();
    float dt_sum = 0.0f;
    int dt_count = 0;

    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_time) * 1e-6f; // seconds
        last_time = now;

        if (dt < 0.0005f) dt = 0.0005f;   // min 0.5 ms
        if (dt > 0.0050f) dt = 0.0050f;   // max 5.0 ms

        dt_sum += dt;
        dt_count++;
        
        if ((now - last_print_time) >= 1000000) { // 1 second in microseconds
            /*
            float avg_dt = dt_sum / dt_count;
            ESP_LOGI("CONTROL", "Average dt over last second: %.6f s", avg_dt);
            dt_sum = 0.0f;
            dt_count = 0;
            */
           ESP_LOGE(PIDTAG,"%f", pid_roll.integral);
            last_print_time = now;
           
        }

        // --- Sensor read + PID + motor update ---
        mpu_raw_t raw_data;
        mpu_rates_t rates = {0};
        if (mpu_read_raw(&raw_data) == ESP_OK) {
            sensor_error_count = 0;
            //angles = mpu_get_filtered_angles(raw_data, angles, dt);
            rates = mpu_get_rates(raw_data);
             ESP_LOGE(PIDTAG,"%f %f %f", rates.rate_roll, rates.rate_pitch, rates.rate_yaw);
        }
        else {
            sensor_error_count++;
            if (sensor_error_count > 10) { // z.B. 100ms Fehler
                //ESP_LOGE("MAIN", "Sensor mehrfach ausgefallen, Motoren aus!");
                killswitch = true;
            }
        }
        
        PID_SetTunings(&pid_roll,rollp,rolli,rolld);
        PID_SetTunings(&pid_pitch,pitchp,pitchi,pitchd);
        PID_SetTunings(&pid_yaw,yawp,yawi,yawd);
        
        float roll_output  = PID_Compute(&pid_roll,  set_rate_roll, rates.rate_roll, dt); 
        float pitch_output = PID_Compute(&pid_pitch, set_rate_pitch, rates.rate_pitch, dt);
        float yaw_output   = PID_Compute(&pid_yaw,   set_rate_yaw, rates.rate_yaw,  dt);        

        m0 = baseThrottle + roll_output + pitch_output + yaw_output;  // front-left
        m1 = baseThrottle - roll_output + pitch_output - yaw_output;  // front-right
        m2 = baseThrottle - roll_output - pitch_output + yaw_output;  // rear-right
        m3 = baseThrottle + roll_output - pitch_output - yaw_output;  // rear-left

        
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

            // PID-Integrator Rücksetzen
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
    // Initialize motors
    Motor_Init();

    // Initialize MPU6050
    if (mpu_init() != ESP_OK) {
        //ESP_LOGE("MAIN", "Failed to initialize MPU6050");
        return;
    }
    mpu_calibrate_gyro();
    
    PID_Init(&pid_roll, 0.5f, 0.0f, 0.0f); //konstanten
    PID_Init(&pid_pitch, 0.5f, 0.00f, 0.0f); //kostanten
    PID_Init(&pid_yaw, 0.0f, 0.00f, 0.0f); //kostanten

    
    PID_SetOutputLimits(&pid_roll, -300, 300);
    PID_SetOutputLimits(&pid_pitch, -300, 300);
    PID_SetOutputLimits(&pid_yaw, -150, 150);
    
    xTaskCreatePinnedToCore(control_task, "control_task", 8192, NULL, 9, NULL, 0);

    // optionally keep app_main busy or delete task:
    vTaskDelete(NULL);  
    /*
    */
    
}
