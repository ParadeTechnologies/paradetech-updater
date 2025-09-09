/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "dut_driver.h"

#define KERNEL_DRIVER_PATH  "/sys/bus/i2c/drivers"
#define KERNEL_MODULE_LINE_MAX_LEN  256

char* DUT_DRIVER_NAMES[] = {
		"TTDL",
		"I2C-HID Linux Driver",
		"No valid DUT driver found"
};

char* DUT_DRIVER_ADAPTER_NAMES[NUM_OF_DUT_DRIVERS] = {
        "pt_i2c_adapter",
        "i2c_hid",
		""
};

static bool driver_found = false;
static DUT_Driver driver = DUT_DRIVER_ERROR;

DUT_Driver get_dut_driver()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	DIR *dir = NULL;
	struct dirent *de;

	if (driver_found) {
		output(DEBUG, "Already determined that %s is active.\n",
				DUT_DRIVER_NAMES[driver]);
		goto RETURN;
	}

	errno = 0;
	dir = opendir(KERNEL_DRIVER_PATH);
	if (dir == NULL) {
		output(ERROR, "%s: Failed to open \"%s\". %s [%d].\n", __func__,
				KERNEL_DRIVER_PATH, strerror(errno), errno);
		return DUT_DRIVER_ERROR;
	}

	while ((de = readdir(dir)) != NULL) {
		if (strcmp(de->d_name, ".") == 0 ||
		    strcmp(de->d_name, "..") == 0) {
			continue;
		}

		output(DEBUG, "%s is found\n", de->d_name);

		if (0 == strncmp(
				DUT_DRIVER_ADAPTER_NAMES[DUT_DRIVER_I2C_HID],
				de->d_name,
				strlen(DUT_DRIVER_ADAPTER_NAMES[DUT_DRIVER_I2C_HID]))) {
			driver = DUT_DRIVER_I2C_HID;
			driver_found = true;
		} else if (0 == strncmp(
				DUT_DRIVER_ADAPTER_NAMES[DUT_DRIVER_TTDL],
				de->d_name,
				strlen(DUT_DRIVER_ADAPTER_NAMES[DUT_DRIVER_TTDL]))) {
			driver = DUT_DRIVER_TTDL;
			driver_found = true;
		}
		if (driver_found) {
			output(DEBUG, "Using %s to drive the DUT.\n",
					DUT_DRIVER_NAMES[driver]);
		}
	}

	closedir(dir);

	if (!driver_found) {
		driver = DUT_DRIVER_I2C_HID;
		driver_found = true;
		output(
		    DEBUG,
		    "No driver found, attempting to use %s to drive the DUT.\n",
		    DUT_DRIVER_NAMES[driver]);
	}

RETURN:
	output(DEBUG, "%s: Leaving.\n", __func__);
	return driver;
}
