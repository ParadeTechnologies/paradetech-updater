/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_CHANNEL_H_
#define PTLIB_CHANNEL_H_

#include "../hid/hid.h"
#include "../report_data.h"
#include "../file/ptlib_file.h"

typedef enum {
	CHANNEL_TYPE_NONE,
	CHANNEL_TYPE_HIDRAW,
	CHANNEL_TYPE_I2CDEV,
	CHANNEL_TYPE_TTDL,
	NUM_OF_CHANNEL_TYPES
} ChannelType;

extern char* CHANNEL_TYPE_NAMES[NUM_OF_CHANNEL_TYPES];

typedef struct {
	ChannelType type;
	int (*setup)(HID_Report_ID report_id);
	int (*get_hid_descriptor)(HID_Descriptor* hid_desc);
	int (*send_report)(const ReportData* report);
	Poll_Status (*get_report)(ReportData* report, bool apply_timeout,
			long double timeout_val);
	int (*teardown)();
} Channel;

#endif
