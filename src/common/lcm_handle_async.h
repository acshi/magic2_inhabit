#pragma once

#include <lcm/lcm.h>

// Based on https://github.com/lcm-proj/lcm/blob/master/examples/c/listener-async.c#L59
void lcm_handle_async(lcm_t *lcm);
