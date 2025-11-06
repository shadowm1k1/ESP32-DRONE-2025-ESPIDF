#pragma once

#include "driver/ledc.h"

#define MOTOR_COUNT 4

// Initialize the motors (sets up PWM)
void Motor_Init(void);

// Set duty cycle of all motors (0-100%)
void Motor_SetDuty(uint16_t val, uint8_t num);
