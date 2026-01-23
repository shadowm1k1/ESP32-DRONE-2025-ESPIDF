#include "../include/pid.h"

extern float contthrottle;
void PID_Init(PID_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->outMin = -30.0f;
    pid->outMax = 30.0f;
    pid->integralMin = 0.0f;
    pid->integralMax = 0.0f;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

     if (ki > 0.0f) {
        pid->integralMax = pid->outMax / ki;
        pid->integralMin = pid->outMin / ki;
    }
}

void PID_SetOutputLimits(PID_t *pid, float min, float max) {
    pid->outMin = min;
    pid->outMax = max;
}

float PID_Compute(PID_t *pid, float setpoint, float measured, float dt) {

    float error = setpoint - measured;
    
    pid->integral += error * dt;
    

    if (pid->integral > pid->integralMax)
        pid->integral = pid->integralMax;
    else if (pid->integral < pid->integralMin)
        pid->integral = pid->integralMin;

    float derivative = (error - pid->prevError) / dt;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->outMax) output = pid->outMax;
    else if (output < pid->outMin) output = pid->outMin;

    pid->prevError = error;
    return output;
}

void printConsts(PID_t *pid)
{
    
    //ESP_LOGI(PIDTAG, " p: %6.2f | i: %6.2f | d: %6.2f", pid->kp, pid->ki,pid->kd);
}