#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "common/time_util.h"
#include "lcm/lcm.h"

int main(int argc, char *argv[])
{
    lcm_t *lcm = lcm_create(NULL);
    if (!lcm) {
        fprintf(stderr, "LCM Failed to initialize. Aborting.\n");
        return 1;
    }



    while (1) {
        lcm_handle(lcm);
    }

    lcm_destroy(lcm);
    return 0;
}
