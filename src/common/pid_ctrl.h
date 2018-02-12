#pragma once

#include <stdbool.h>
#include "median_filter.h"

typedef struct pid_ctrl {

    float kb; // Proportional set point (bias) Gain
    float kp; // Proportional error Gain
    float ki; // Integral Gain
    float kd; // Derivative Gain
    float bp_scale; // scalar gain for b and p gains

    float pidInput;
    float pidOutput;

    float bTerm;   // proportional bias term (based on set point)
    float pTerm;   // proportional term of output (based on error)
    float iTerm;   // integral term
    float dTerm;   // derivative term

    // derivative debugging
    float dRaw;
    float dMedianOut;
    float dOut;

    float resetIThresh; // difference in set point at which iTerm is cleared
    float setPoint; // Target value to reach

    float iTermMin, iTermMax; // integral saturation
    float outputMin, outputMax; //pid output saturation

    int medianN;
    float cutoffHz;

    // rc_filter_t dFilter; //low pass filter for DTerm
    float updateHz; // rate in Hz of pid loop update

    median_filter_t *dMedianFilter;
} pid_ctrl_t;


//Initialize the PID controller with gains
pid_ctrl_t *pid_init(
    float kb,
    float kp,
    float ki,
    float kd,
    int medianN,
    float cutoffHz,
    float updateHz
);

float pid_compute(pid_ctrl_t *pid, float error);

void pid_set_setpoint(pid_ctrl_t *pid, float setPoint);

void pid_set_tunings(pid_ctrl_t *pid,
    float kb,
    float kp,
    float ki,
    float kd
);

void pid_set_output_limits(pid_ctrl_t *pid,
    float min,
    float max
);

void pit_set_integral_limits(pid_ctrl_t *pid,
    float min,
    float max
);

void pid_reset_integrator(pid_ctrl_t *pid);

void pid_set_derivative_filter(pid_ctrl_t *pid,
    float dFilterHz
);

void pid_set_update_rate(pid_ctrl_t *pid,
    float updateHz
);
