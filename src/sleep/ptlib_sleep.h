/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef _PTLIB_SLEEP_H
#define _PTLIB_SLEEP_H

#include <sys/time.h>
#include "../logging.h"

#define USEC_SEC_RATIO (long double) 1e6

typedef unsigned char uint8_t;

extern void sleep_ms (unsigned int ms);
extern void sleep_us (unsigned int us);
extern bool time_limit_reached(const struct timeval* start, long double limit);

#endif
