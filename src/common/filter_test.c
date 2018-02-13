#include <stdio.h>
#include <stdlib.h>

#include "median_filter.h"
#include "chebyshev_filter.h"
#include "pid_ctrl.h"

int main(int argc, char **argv)
{
    // median_filter_t *filter = median_filter_create(3);
    // float vals[] = {1, 2, 3, 10, 0, 1, 1, -1, 0, -1};
    // for (int i = 0; i < (sizeof(vals)/sizeof(float)); i++) {
    //     float out = median_filter_march(filter, vals[i]);
    //     printf("%.1f\n", out);
    // }

    float vals[1000];
    for (int i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        vals[i] = (float)rand() / (float)RAND_MAX;
    }

    chebyshev_filter_t *c_filter = chebyshev_lowpass_create(20, 1.0/100, 6, 0.2);
    for (int i = 0; i < (sizeof(vals)/sizeof(float)); i++) {
        double out = chebyshev_filter_march(c_filter, vals[i]);
        printf("%.5f\n", out);
    }
    return 0;
}
