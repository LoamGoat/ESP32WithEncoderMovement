#ifndef PID_H
#define PID_H

struct PID {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float prevError;
    float outputMin;
    float outputMax;
};

void pidInit(PID* pid, float Kp, float Ki, float Kd, float min_out, float max_out);
float pidCompute(PID* pid, float input, float dt);
#endif