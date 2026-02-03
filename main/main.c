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
mpu_rates_t rates = {0};

float m0, m1, m2, m3;
volatile float baseThrottle = 0;
volatile bool killswitch = true;
volatile float contthrottle, controll, contpitch, contyaw;
volatile float rollp,rolli,rolld,pitchp,pitchi,pitchd,yawp,yawi,yawd;
volatile float ControlLoopFrequency = 0;


/*----inner loop ----*/
PID_t pid_roll_inner, pid_pitch_inner, pid_yaw_inner;

volatile float set_rate_roll = 0.0f; 
volatile float set_rate_pitch = 0.0f;
volatile float set_rate_yaw = 0.0f;

/*------------------------------------------------------------*/

/*
// ----------  outer-loop instances  ---------- 
PID_t pid_roll_angle, pid_pitch_angle,pid_yaw_angle;

static float target_roll_angle  = 0.0f;
static float target_pitch_angle = 0.0f;
static float target_yaw_angle   = 0.0f;

volatile float target_yaw_rate = 0.0f; 

*/


void control_task(void *pvParameters)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(1);
    TickType_t last_wake_time = xTaskGetTickCount(); // für 1000hz loop
    

    int64_t last_time = esp_timer_get_time();//für dt berechnen
    static int sensor_error_count = 0;

    static int64_t last_angle_us = 0; // timer für 250hz for angles
    static int64_t set_pid_konstants_lasttime = 0; // timer für kosntanten

    static float dt_sum = 0.0f;
    static uint32_t dt_count = 0;
    static int64_t last_print_us = 0;

    while (1) {
        int64_t now = esp_timer_get_time(); //elapsed time for dt
        float dt = (now - last_time) * 1e-6f; // seconds
        last_time = now;
        dt_sum += dt;
        dt_count++; 
        
        if (now - last_print_us >= 1000000) { // 1 second
            last_print_us = now;

            float avg_dt = 0.0f;
            if (dt_count > 0) {
                avg_dt = dt_sum / dt_count;
            }

            float freq = (avg_dt > 0) ? (1.0f / avg_dt) : 0.0f;
            ControlLoopFrequency = freq;

            dt_sum = 0.0f;
            dt_count = 0;
        }

        set_rate_pitch = contpitch; // controlerwerte als gewunschte pos setzen
        set_rate_roll = controll;
        set_rate_yaw = contyaw;


        // --- Sensor read + PID + motor update ---
        mpu_raw_t raw_data;
        
        if (mpu_read_raw(&raw_data) == ESP_OK) {
            sensor_error_count = 0;
            //angles = mpu_get_filtered_angles(raw_data, angles, dt);
            rates = mpu_get_rates(raw_data);
        }
        else {//sensor not read
            sensor_error_count++;
            if (sensor_error_count > 10) {
                killswitch = true;
            }
        }
        
       if(now - set_pid_konstants_lasttime >= 100000) //1000000 for second ||| 100000 for 100ms 
       {
            set_pid_konstants_lasttime = now;
            PID_SetTunings(&pid_roll_inner,rollp,rolli,rolld);
            PID_SetTunings(&pid_pitch_inner,pitchp,pitchi,pitchd);
            PID_SetTunings(&pid_yaw_inner,yawp,yawi,yawd);
       }

       /*
        if (now - last_angle_us >= 4000) { // 250 Hz
            last_angle_us = now;

            // compute desired rates from angle error 
            set_rate_roll  = PID_Compute(&pid_roll_angle,  target_roll_angle,  angles.roll,  dt);
            set_rate_pitch = PID_Compute(&pid_pitch_angle, target_pitch_angle, angles.pitch, dt);
            set_rate_yaw   = target_yaw_rate; 

        }
        */

       /*---------- innerer loop von reglern ------------*/
        float roll_output  = PID_Compute(&pid_roll_inner,  set_rate_roll, rates.rate_roll, dt); 
        float pitch_output = PID_Compute(&pid_pitch_inner , set_rate_pitch, rates.rate_pitch, dt);
        float yaw_output   = PID_Compute(&pid_yaw_inner,   set_rate_yaw, rates.rate_yaw,  dt);        

        /*
        float throttle_factor = contthrottle / 1000;
        throttle_factor = fmaxf(throttle_factor, 0.2f); // niemals 0
        
        roll_output  *= throttle_factor;
        pitch_output *= throttle_factor;
        yaw_output   *= throttle_factor;
       */

        m0 = contthrottle + roll_output + pitch_output + yaw_output;  // front-left
        m1 = contthrottle - roll_output + pitch_output - yaw_output;  // front-right
        m2 = contthrottle - roll_output - pitch_output + yaw_output;  // rear-right
        m3 = contthrottle + roll_output - pitch_output - yaw_output;  // rear-left

        /*
        float min_m = fminf(fminf(fminf(m0, m1), m2), m3);
        float max_m = fmaxf(fmaxf(fmaxf(m0, m1), m2), m3);

        float offset = 0.0f;
        if (max_m > 1023.0f) offset = 1023.0f - max_m;

        else if (min_m < 0.0f) offset = -min_m;
        m0 += offset; m1 += offset; m2 += offset; m3 += offset;
       */

       /*------------- motor min und max setzen ----------- */
       if(m0 <0.0f ) m0 = 0.0f; 
       if(m0 > 1023.0f) m0 = 1023.0f;
       if(m1 <0.0f ) m1 = 0.0f; 
       if(m1 > 1023.0f) m1 = 1023.0f;
       if(m2 <0.0f ) m2 = 0.0f; 
       if(m2 > 1023.0f) m2 = 1023.0f;
       if(m3 <0.0f ) m3 = 0.0f; 
       if(m3 > 1023.0f) m3 = 1023.0f;

        /*-------------- wenn controler in nahe 0 motoren aus ---------*/
        if(contthrottle < 30)
        {
            m0 = 0;
            m1 = 0;
            m2 = 0;
            m3 = 0;
        }

        if (!killswitch) {
            Motor_SetDuty((uint16_t)m0, 0); //oben links cw
            Motor_SetDuty((uint16_t)m1, 1); //oben rechts ccw
            Motor_SetDuty((uint16_t)m2, 2); // unten rechts cw
            Motor_SetDuty((uint16_t)m3, 3); // unten links ccw
        } else {
            
            Motor_SetDuty(0, 0);
            Motor_SetDuty(0, 1);
            Motor_SetDuty(0, 2);
            Motor_SetDuty(0, 3);

            // PID-Integrator Rücksetzen
            pid_roll_inner.integral = 0;
            pid_pitch_inner.integral = 0;
            pid_yaw_inner.integral = 0;

            /*
            pid_roll_angle.integral  = 0;
            pid_pitch_angle.integral = 0;
            pid_yaw_angle.integral   = 0;
            */
        }
        
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}


void app_main(void)
{
    wifi_start();
    // Initialize motors
    Motor_Init();

    // Initialize MPU6050
    if (mpu_init() != ESP_OK) {
        //ESP_LOGE("MAIN", "Failed to initialize MPU6050");
        return;
    }
    mpu_calibrate_gyro();
    
    //rates pid configuraton inner loop
    PID_Init(&pid_roll_inner, 0.0f, 0.0f, 0.0f); //konstanten
    PID_Init(&pid_pitch_inner, 0.0f, 0.0f, 0.0f); //kostanten
    PID_Init(&pid_yaw_inner, 0.0f, 0.00f, 0.0f); //kostanten
    
    PID_SetOutputLimits(&pid_roll_inner, -300, 300);
    PID_SetOutputLimits(&pid_pitch_inner, -300, 300);
    PID_SetOutputLimits(&pid_yaw_inner, -150, 150);

    //angle  pid config outer loop
    /*
    PID_Init(&pid_roll_angle,  0.5f, 0.0f, 0.0f);
    PID_Init(&pid_pitch_angle, 0.0f, 0.0f, 0.0f);
    PID_Init(&pid_yaw_angle,   0.0f, 0.0f, 0.0f);

    PID_SetOutputLimits(&pid_pitch_angle, -50.0f, 50.0f);
    PID_SetOutputLimits(&pid_roll_angle,  -50.0f, 50.0f);
    PID_SetOutputLimits(&pid_yaw_angle,  -30.0f, 30.0f);
    */
    
    xTaskCreatePinnedToCore(control_task, "control_task", 8192, NULL, 9, NULL, 0);

    // optionally keep app_main busy or delete task:
    vTaskDelete(NULL);  
    
}
