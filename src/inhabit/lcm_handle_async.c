#define _XOPEN_SOURCE 700
#include <stdio.h>
#include <inttypes.h>
#include <sys/select.h>
#include <time.h>
#include "lcm_handle_async.h"

// Based on https://github.com/lcm-proj/lcm/blob/master/examples/c/listener-async.c#L59
void lcm_handle_async(lcm_t *lcm)
{
    int lcm_fd = lcm_get_fileno(lcm);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcm_fd, &fds);

    struct timeval timeout = {0, 0};

    int status = select(lcm_fd + 1, &fds, NULL, NULL, &timeout);
    if(0 != status && FD_ISSET(lcm_fd, &fds)) {
        lcm_handle(lcm);
    }
}
