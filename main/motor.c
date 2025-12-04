#include "../include/motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PWM_RESOLUTION 1023

static int motor_pins[MOTOR_COUNT] = {6, 3, 4, 5}; //6 --> topleft 3 --> topright 4 --> botright 5 --> botleft

void Motor_Init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num       = motor_pins[i],
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = i,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0
        };
        ledc_channel_config(&ledc_channel);
    }
}


void Motor_SetDuty(uint16_t val, uint8_t num)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, num, val);      // num = LEDC channel (0..3)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, num);
}
