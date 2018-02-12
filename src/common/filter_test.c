#include <stdio.h>

#include "median_filter.h"
#include "pid_ctrl.h"

int main(int argc, char **argv)
{
    median_filter_t *filter = median_filter_create(3);
    float vals[] = {1, 2, 3, 10, 0, 1, 1, -1, 0, -1};
    for (int i = 0; i < (sizeof(vals)/sizeof(float)); i++) {
        float out = median_filter_march(filter, vals[i]);
        printf("%.1f\n", out);
    }
    return 0;
}
