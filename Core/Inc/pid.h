#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;             // Proportional gain
    float ki;             // Integral gain
    float kd;             // Derivative gain
    float setpoint;       // Desired value
    float integral;       // Integral term
    float previous_error; // Previous error term
    float max_control;    // Maximum control output
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float setpoint, float max_control);
float PID_Compute(PID_Controller *pid, float measured_value, float dt);
void PID_Reset(PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif // PID_H