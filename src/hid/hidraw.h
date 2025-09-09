/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef HIDRAW_H_
#define HIDRAW_H_

#include <fcntl.h>
#include <linux/hidraw.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include "../channel/channel.h"
#include "../file/ptlib_file.h"
#include "../logging.h"
#include "../report_data.h"
#include "../sleep/ptlib_sleep.h"
#include "hid.h"

#define HIDRAW0_SYSFS_NODE_FILE "/dev/hidraw0"

extern Channel hidraw_channel;

extern int auto_detect_hidraw_sysfs_node(int vendor_id, int product_id);
extern void clear_hidraw_report_buffer();
extern int get_hid_descriptor_from_hidraw(HID_Descriptor* hid_desc);
extern Poll_Status get_report_from_hidraw(ReportData* report,
		bool apply_timeout, long double timeout_val);
extern int get_report_descriptor_from_hidraw(ReportData* report);
extern int init_hidraw_api(const char* sysfs_node_file,
	const HID_Descriptor* hid_desc);
extern int init_input_report(ReportData* report);
extern int send_report_via_hidraw(const ReportData* report);
extern int start_hidraw_report_reader(HID_Report_ID report_id);
extern int stop_hidraw_report_reader();
extern int rebind_hid_driver(int vid, int pid);
#endif 
