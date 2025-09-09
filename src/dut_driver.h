/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef DUT_DRIVER_H_
#define DUT_DRIVER_H_

#include "logging.h"

typedef enum {
        DUT_DRIVER_TTDL,
        DUT_DRIVER_I2C_HID,
		DUT_DRIVER_ERROR,
        NUM_OF_DUT_DRIVERS
} DUT_Driver;

extern char* DUT_DRIVER_NAMES[NUM_OF_DUT_DRIVERS];

DUT_Driver get_dut_driver();

#endif
