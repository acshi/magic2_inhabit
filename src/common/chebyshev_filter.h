#pragma once

#include <stdbool.h>
#include "common/zarray.h"

typedef struct chebyshev_filter {
    zarray_t *coefs; // of filter_coefs_t
    zarray_t *last_as; // of double[3], one for each filter_coefs_t
    zarray_t *last_bs; // of double[3], one for each filter_coefs_t
} chebyshev_filter_t;

typedef struct filter_coefs {
    double as[3];
    double bs[3];
} filter_coefs_t;

chebyshev_filter_t *chebyshev_create(double hz, double dt, int poles, double ripple, bool highpass);
void chebyshev_filter_destroy(chebyshev_filter_t *f);
double chebyshev_filter_march(chebyshev_filter_t *f, double v);

static inline chebyshev_filter_t *chebyshev_lowpass_create(double hz, double dt, int poles, double ripple)
{
    return chebyshev_create(hz, dt, poles, ripple, false);
}

static inline chebyshev_filter_t *chebyshev_highpass_create(double hz, double dt, int poles, double ripple)
{
    return chebyshev_create(hz, dt, poles, ripple, true);
}

static inline chebyshev_filter_t *butterworth_lowpass_create(double hz, double dt, int poles)
{
    return chebyshev_create(hz, dt, poles, 0.0, false);
}

static inline chebyshev_filter_t *butterworth_highpass_create(double hz, double dt, int poles)
{
    return chebyshev_create(hz, dt, poles, 0.0, true);
}
