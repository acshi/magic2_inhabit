#include "chebyshev_filter.h"
#include <math.h>

static void normalizeFilterGain(filter_coefs_t *stage, bool highpass) {
    double sumA = 0.0;
    double sumB = 0.0;
    int n = sizeof(stage->as) / sizeof(stage->as[0]);
    for (int i = 0; i < n; i++) {
        if (highpass) {
            if ((i % 2) == 0) {
                sumA += stage->as[i];
                sumB += stage->bs[i];
            } else {
                sumA -= stage->as[i];
                sumB -= stage->bs[i];
            }
        } else {
            sumA += stage->as[i];
            sumB += stage->bs[i];
        }
    }

    double gain = sumA / (1 - sumB);
    for (int i = 0; i < n; i++) {
        stage->as[i] /= gain;
    }
}

// Chebyshev biquad (2-poles) recursive coefficients
// Adapted from The Scientist and Engineer's Guide to Digital Signal Processing, Steven W. Smith
// poleIndex = [0, poleCount)
// percentRipple in the pass band can range from 0 for a butterworth to about 0.29. Something like 0.005 is a good trade-off.
static filter_coefs_t chebyshevBiquad(double freq, double ripple, int poleIndex, int poleCount, bool highpass) {
    // We start off by designing a low-pass filter with unity cut-off frequency

    // Location of pole on unit circle, real and imaginary parts
    // The maximally flat butterworth filter positions the poles so that
    // they form a semi-circle on the left side of the s-plane (sigma < 0)
    // The half offset keeps the poles evenly spaced and off of the sigma=0 line
    // s-plane s = sigma + i * omega = poleR + i * poleI
    double angle = (poleIndex + 0.5) * M_PI / poleCount;
    double poleI = sin(angle);
    double poleR = -cos(angle);

    // The chebyshev filter uses an ellipse to move all of the poles closer to the sigma=0 line
    // This causes pass-band ripple and sharpens the drop off
    // Warp coordinates from being on a circle to an ellipse
    if (ripple != 0.0) {
        double e = sqrt(1.0 / ((1.0 - ripple) * (1.0 - ripple)) - 1.0);
        double v = asinh(1 / e) / poleCount;
        double k = acosh(1 / e) / poleCount;

        k = cosh(k);

        poleR *= sinh(v) / k;
        poleI *= cosh(v) / k;
    }

    // bilinear s-domain to z-domain transformation
    double t = 2 * tan(0.5);
    double w = 2 * M_PI * freq;
    double m = poleR * poleR + poleI * poleI;
    double d = 4 - 4 * poleR * t + m * t * t;
    double x0 = t * t / d;
    double x1 = 2 * t * t / d;
    double x2 = t * t / d;
    double y1 = (8 - 2 * m * t * t) / d;
    double y2 = (-4 - 4 * poleR * t - m * t * t) / d;

    // We now have the coefficients of a low-pass filter with a cutoff frequency of 1 (2 times the nyquist)...
    // We must now apply our desired frequency and convert to a high-pass filter if necessary
    // as with the bilinear tranform, these are the results of a substitution in the transfer function...

    double k;
    if (highpass) {
        k = -cos(w / 2 + 0.5) / cos(w / 2 - 0.5);
    } else {
        k = sin(0.5 - w / 2) / sin(0.5 + w / 2);
    }

    d = 1 + (y1 * k - y2 * k * k);
    double a0 = (x0 - x1 * k + x2 * k * k) / d;
    double a1 = (-2 * x0 * k + x1 + (x1 * k * k - 2 * x2 * k)) / d;
    double a2 = (x0 * k * k - x1 * k + x2) / d;
    double b1 = (2 * k + y1 + y1 * k * k - 2 * y2 * k) / d;
    double b2 = (-k * k - y1 * k + y2) / d;
    if (highpass) {
        a1 = -a1;
        b1 = -b1;
    }

    // we now have the desired coefficients of our low/high pass filter with the desired cutoff frequency
    // however, the gain has not been normalized, if that is desired...
    return (filter_coefs_t) { {a0, a1, a2}, {0, b1, b2} };
}

// Chebyshev recursive coefficients
// Adapted from The Scientist and Engineer's Guide to Digital Signal Processing, Steven W. Smith
chebyshev_filter_t *chebyshev_create(double hz, double dt, int poles, double ripple, bool highpass)
{
    assert(poles >= 2 && poles <= 20 && (poles % 2) == 0);
    assert(dt > 0);
    assert(hz > 0);
    assert(ripple >= 0 && ripple <= 0.3);

    chebyshev_filter_t *f = calloc(1, sizeof(chebyshev_filter_t));
    f->coefs = zarray_create(sizeof(filter_coefs_t));
    f->last_as = zarray_create(sizeof(double[3]));
    f->last_bs = zarray_create(sizeof(double[3]));

    double freq = hz * dt;

    for (int i = 0; i < poles / 2; i++) {
        filter_coefs_t stage = chebyshevBiquad(freq, 0.005, i, poles, highpass);
        normalizeFilterGain(&stage, highpass);
        zarray_add(f->coefs, &stage);
        double last_vals[3] = { 0 };
        zarray_add(f->last_as, last_vals);
        zarray_add(f->last_bs, last_vals);
    }

    return f;
}

void chebyshev_filter_destroy(chebyshev_filter_t *f)
{
    zarray_destroy(f->coefs);
    zarray_destroy(f->last_as);
    zarray_destroy(f->last_bs);
    free(f);
}

double chebyshev_filter_march(chebyshev_filter_t *f, double v)
{
    double a0 = v;
    double b0 = 0.0;

    // run through each stage of coefficients
    for (int j = 0; j < zarray_size(f->coefs); j++) {
        filter_coefs_t *coefs;
        double *last_as;
        double *last_bs;
        zarray_get_volatile(f->coefs, j, &coefs);
        zarray_get_volatile(f->last_as, j, &last_as);
        zarray_get_volatile(f->last_bs, j, &last_bs);

        // shift the old a and b values..
        for (int i = 2; i > 0; i--) {
            last_as[i] = last_as[i - 1];
            last_bs[i] = last_bs[i - 1];
        }

        last_as[0] = a0;

        b0 = 0.0;
        for (int i = 0; i < 3; i++) {
            b0 += coefs->as[i] * last_as[i];
        }
        for (int i = 0; i < 3; i++) {
            b0 += coefs->bs[i] * last_bs[i];
        }

        last_bs[0] = b0;
        a0 = b0;
    }

    return b0;
}
