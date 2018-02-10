#include "pid.h"

#include <stdlib.h>

pid_t *pid_init(float Kb, float Kp, float Ki, float Kd, int medianN, float cutoffHz, float updateHz) {
    // zero initial values
    pid_t *pid =  calloc(1, sizeof(pid_t));
    pid->bp_scale = 1.0;

    // for removing outliers
    if (medianN > 1) {
        initMedianFilter(&pid->dMedianFilter, medianN);
    }

    // moving average is optimal among low-pass filters for removing noise
    // (The Scientist and Engineer's Guide to Digital Signal Processing, page 278-279)
    if (cutoffHz > 0) {
        //rc_moving_average(&pid->dFilter, movingN, 1.0);
        rc_butterworth_lowpass(&pid->dFilter, 4, DT, 2*PI*cutoffHz);
    }

    pid->medianN = medianN;
    pid->cutoffHz = cutoffHz;

    pid->updateHz = updateHz;
    pid_set_tunings(pid, Kb, Kp, Ki, Kd);

    // printf("Constructed pid with %.2f %.2f %.2f %.2f\n", Kb, Kp, Ki, Kd);

    pid->resetIThresh = -1;
    return pid;
}

static float constrain(float in, float min, float max) {
    if (in < min) {
        return min;
    } else if (in > max) {
        return max;
    }
    return in;
}

float pid_compute(pid_t *pid, float newVal) {
    float prevError = pid->setPoint - pid->pidInput;
    float error = pid->setPoint - newVal;

    pid->bTerm = pid->kb * pid->bp_scale * pid->setPoint;
    pid->pTerm = pid->kp * pid->bp_scale * error;
    pid->iTerm += pid->ki * pid->bp_scale * error;
    pid->iTerm = constrain(pid->iTerm, pid->iTermMin, pid->iTermMax);
    pid->dRaw = error - prevError;
    if (pid->medianN <= 1) {
        pid->dMedianOut = pid->dRaw;
    } else {
        pid->dMedianOut = marchMedianFilter(&pid->dMedianFilter, pid->dRaw);
    }
    if (pid->cutoffHz <= 0) {
        pid->dOut = pid->dMedianOut;
    } else {
        pid->dOut = rc_march_filter(&pid->dFilter, pid->dMedianOut);
    }
    pid->dTerm = pid->kd * pid->dOut;

    pid->pidInput = newVal;
    float rawOut = pid->bTerm + pid->pTerm + pid->iTerm + pid->dTerm;
    pid->pidOutput = constrain(rawOut, pid->outputMin, pid->outputMax);
    return pid->pidOutput;
}

void pid_set_setpoint(pid_t *pid, float setPoint) {
    if (pid->resetIThresh >= 0 && (fabs(setPoint - pid->setPoint) > (pid->resetIThresh / pid->updateHz))) {
        pid_reset_integrator(pid);
    }
    pid->setPoint = setPoint;
}

void pid_set_tunings(pid_t *pid, float Kb, float Kp, float Ki, float Kd) {
    //scale gains by update rate in seconds for proper units
    //note we are using hz and not dt.
    pid->kb = Kb;
    pid->kp = Kp;
    pid->ki = Ki / pid->updateHz;
    pid->kd = Kd * pid->updateHz;
}

void pid_set_output_limits(pid_t *pid, float min, float max){
    pid->outputMin = min;
    pid->outputMax = max;
}

void pid_set_integral_limits(pid_t *pid, float min, float max){
    pid->iTermMin = min;
    pid->iTermMax = max;
}

void pid_reset_integrator(pid_t *pid){
    pid->iTerm = 0;
}

void pid_set_update_rate(pid_t *pid, float updateHz){
    float Kb = pid->kb;
    float Kp = pid->kp;
    float Ki = pid->ki * pid->updateHz;
    float Kd = pid->kd / pid->updateHz;
    pid->updateHz = updateHz;
    pid_set_tunings(pid, Kb, Kp, Ki, Kd);
}
