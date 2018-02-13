#pragma once

#include <stdbool.h>
#include "median_filter.h"
#include "chebyshev_filter.h"

typedef struct pid_ctrl {

    double kb; // Proportional set point (bias) Gain
    double kp; // Proportional error Gain
    double ki; // Integral Gain
    double kd; // Derivative Gain
    double bp_scale; // scalar gain for b and p gains

    double pidInput;
    double pidOutput;

    double bTerm;   // proportional bias term (based on set point)
    double pTerm;   // proportional term of output (based on error)
    double iTerm;   // integral term
    double dTerm;   // derivative term

    // derivative debugging
    double dRaw;
    double dMedianOut;
    double dOut;

    double resetIThresh; // difference in set point at which iTerm is cleared
    double setPoint; // Target value to reach

    double iTermMin, iTermMax; // integral saturation
    double outputMin, outputMax; //pid output saturation

    chebyshev_filter_t *inputFilter; //low pass filter for input values
    chebyshev_filter_t *dFilter; //low pass filter for DTerm
    double updateHz; // rate in Hz of pid loop update

    median_filter_t *dMedianFilter;
} pid_ctrl_t;


//Initialize the PID controller with gains
pid_ctrl_t *pid_create(double kb, double kp, double ki, double kd, double updateHz);

void pid_destroy(pid_ctrl_t *pid);

void pid_set_input_filter(pid_ctrl_t *pid, double cutoffHz, int poles, double ripple);
void pid_set_derivative_median_filter(pid_ctrl_t *pid, int medianN);
void pid_set_derivative_filter(pid_ctrl_t *pid, double cutoffHz, int poles, double ripple);

double pid_compute(pid_ctrl_t *pid, double error);
void pid_set_setpoint(pid_ctrl_t *pid, double setPoint);

void pid_set_tunings(pid_ctrl_t *pid, double kb, double kp, double ki, double kd);
void pid_set_output_limits(pid_ctrl_t *pid, double min, double max);
void pid_set_integral_limits(pid_ctrl_t *pid, double min, double max);
void pid_reset_integrator(pid_ctrl_t *pid);
