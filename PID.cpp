#include "PID.h"

void pidInit(PID* pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0;
    pid->integral = 0;
    pid->prevError = 0;
    pid->outputMin = min_out;
    pid->outputMax = max_out;
}
float pidCompute(PID* pid, float input, float dt) {
    float error = pid->setpoint - input;
    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (output > pid->outputMax) {
        output = pid->outputMax;
    }
    if (output < pid->outputMin) {
        output = pid->outputMin;
    }

    pid->prevError = error;
    return output;
}