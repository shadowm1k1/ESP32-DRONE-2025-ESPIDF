#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integral;
    float prevError;
    float prevMeasured;
    float filteredDerivative;
    float outMin, outMax;
    float integralMin, integralMax;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
void PID_SetOutputLimits(PID_t *pid, float min, float max);
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);
void printConsts(PID_t *pid);

#endif