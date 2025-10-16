#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define MOTOR_COUNT 4
int motor_pins[MOTOR_COUNT] = {5, 4, 3, 6};

void app_main(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT, // 10-bit (0–1023)
        .freq_hz          = 5000,               // PWM frequency (5000 Hz)
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num       = motor_pins[i],
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = i,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
    }

    while (true) {
        // Increase from 1% to 10%
        for (int percent = 1; percent <= 10; percent++) {
            uint32_t duty = (1023 * percent) / 100;
            for (int i = 0; i < MOTOR_COUNT; i++) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, i, duty);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        // Decrease back down to 1%
        for (int percent = 10; percent >= 1; percent--) {
            uint32_t duty = (1023 * percent) / 100;
            for (int i = 0; i < MOTOR_COUNT; i++) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, i, duty);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
