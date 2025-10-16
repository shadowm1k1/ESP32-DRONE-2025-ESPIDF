#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../include/motor.h"

void app_main(void)
{
    Motor_Init();

    

    while(1) {
        Motor_SetDuty(10, 1);
        /*
        // Ramp up 1% → 10%
        for (int p = 1; p <= 10; p++) {
            for (int i = 0; i < MOTOR_COUNT; i++) duty[i] = p;
            Motor_SetDuty(duty);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // Ramp down 10% → 1%
        for (int p = 10; p >= 1; p--) {
            for (int i = 0; i < MOTOR_COUNT; i++) duty[i] = p;
            Motor_SetDuty(duty);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        */
    }
}
