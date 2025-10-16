#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
    float outMin;
    float outMax;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
void PID_SetOutputLimits(PID_t *pid, float min, float max);
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);
