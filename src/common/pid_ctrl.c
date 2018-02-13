#include "pid_ctrl.h"

#include <stdlib.h>
#include <math.h>

pid_ctrl_t *pid_create(double Kb, double Kp, double Ki, double Kd, double updateHz) {
    // zero initial values
    pid_ctrl_t *pid =  calloc(1, sizeof(pid_ctrl_t));
    pid->bp_scale = 1.0;

    pid->updateHz = updateHz;
    pid_set_tunings(pid, Kb, Kp, Ki, Kd);

    // printf("Constructed pid with %.2f %.2f %.2f %.2f\n", Kb, Kp, Ki, Kd);

    pid->resetIThresh = -1;
    return pid;
}

void pid_destroy(pid_ctrl_t *pid)
{
    if (pid->inputFilter) {
        chebyshev_filter_destroy(pid->inputFilter);
    }
    if (pid->dMedianFilter) {
        median_filter_destroy(pid->dMedianFilter);
    }
    if (pid->dFilter) {
        chebyshev_filter_destroy(pid->dFilter);
    }
    free(pid);
}

void pid_set_input_filter(pid_ctrl_t *pid, double cutoffHz, int poles, double ripple)
{
    if (pid->inputFilter) {
        chebyshev_filter_destroy(pid->inputFilter);
    }
    pid->inputFilter = chebyshev_lowpass_create(cutoffHz, 1.0/pid->updateHz, poles, ripple);
}

void pid_set_derivative_median_filter(pid_ctrl_t *pid, int medianN)
{
    if (pid->dMedianFilter) {
        median_filter_destroy(pid->dMedianFilter);
    }
    pid->dMedianFilter = median_filter_create(medianN);
}

void pid_set_derivative_filter(pid_ctrl_t *pid, double cutoffHz, int poles, double ripple)
{
    if (pid->dFilter) {
        chebyshev_filter_destroy(pid->dFilter);
    }
    pid->dFilter = chebyshev_lowpass_create(cutoffHz, 1.0/pid->updateHz, poles, ripple);
}

static double constrain(double in, double min, double max) {
    if (in < min) {
        return min;
    } else if (in > max) {
        return max;
    }
    return in;
}

double pid_compute(pid_ctrl_t *pid, double newVal) {
    double filtered_new_val = newVal;
    if (pid->inputFilter) {
        filtered_new_val = chebyshev_filter_march(pid->inputFilter, newVal);
    }

    double prevError = pid->setPoint - pid->pidInput;
    double error = pid->setPoint - filtered_new_val;

    pid->bTerm = pid->kb * pid->bp_scale * pid->setPoint;
    pid->pTerm = pid->kp * pid->bp_scale * error;
    pid->iTerm += pid->ki * pid->bp_scale * error;
    pid->iTerm = constrain(pid->iTerm, pid->iTermMin, pid->iTermMax);
    pid->dRaw = error - prevError;
    if (pid->dMedianFilter) {
        pid->dMedianOut = median_filter_march(pid->dMedianFilter, pid->dRaw);
    } else {
        pid->dMedianOut = pid->dRaw;
    }
    if (pid->dFilter) {
        pid->dOut = chebyshev_filter_march(pid->dFilter, pid->dMedianOut);
    } else {
        pid->dOut = pid->dMedianOut;
    }
    pid->dTerm = pid->kd * pid->dOut;

    pid->pidInput = filtered_new_val;
    double rawOut = pid->bTerm + pid->pTerm + pid->iTerm + pid->dTerm;
    pid->pidOutput = constrain(rawOut, pid->outputMin, pid->outputMax);
    return pid->pidOutput;
}

void pid_set_setpoint(pid_ctrl_t *pid, double setPoint) {
    if (pid->resetIThresh >= 0 && (fabs(setPoint - pid->setPoint) > (pid->resetIThresh / pid->updateHz))) {
        pid_reset_integrator(pid);
    }
    pid->setPoint = setPoint;
}

void pid_set_tunings(pid_ctrl_t *pid, double Kb, double Kp, double Ki, double Kd) {
    //scale gains by update rate in seconds for proper units
    //note we are using hz and not dt.
    pid->kb = Kb;
    pid->kp = Kp;
    pid->ki = Ki / pid->updateHz;
    pid->kd = Kd * pid->updateHz;
}

void pid_set_output_limits(pid_ctrl_t *pid, double min, double max){
    pid->outputMin = min;
    pid->outputMax = max;
}

void pid_set_integral_limits(pid_ctrl_t *pid, double min, double max){
    pid->iTermMin = min;
    pid->iTermMax = max;
}

void pid_reset_integrator(pid_ctrl_t *pid){
    pid->iTerm = 0;
}
