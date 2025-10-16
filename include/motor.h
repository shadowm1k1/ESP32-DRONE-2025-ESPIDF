#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"

#define MOTOR_COUNT 4

// Initialize the motors (sets up PWM)
void Motor_Init(void);

// Set duty cycle of all motors (0-100%)
void Motor_SetDuty(uint8_t percent, uint8_t num);

#endif
