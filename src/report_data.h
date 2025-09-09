/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef REPORT_DATA_H_
#define REPORT_DATA_H_

#include "logging.h"

#define TOUCH_REPORT_VALUE_HEX_FORMAT_STRLEN 3

#define REPORT_DIRECTION_INCOMING_FROM_HOST "Incoming from host"
#define REPORT_DIRECTION_INCOMING_FROM_DUT  "Incoming from DUT"
#define REPORT_DIRECTION_OUTGOING_TO_HOST   "Outgoing to host"
#define REPORT_DIRECTION_OUTGOING_TO_DUT    "Outgoing to DUT"

#define REPORT_FORMAT_HID  "HID"
#define REPORT_FORMAT_PIP1 "PIP1"
#define REPORT_FORMAT_PIP2 "PIP2"
#define REPORT_FORMAT_UART "UART"

#define REPORT_TYPE_COMMAND             "Command"
#define REPORT_TYPE_HID_DESCRIPTOR      "Descriptor"
#define REPORT_TYPE_REPORT_DESCRIPTOR   "Report Descriptor"
#define REPORT_TYPE_RESPONSE            "Response"
#define REPORT_TYPE_UNSOLICTED_RESPONSE "Response (unsolicited)"
#define REPORT_TYPE_TOUCH               "Touch"

typedef enum {
	PROTOCOL_MODE_PIP,
	PROTOCOL_MODE_HID,
} Protocol_Mode;

typedef enum {
	TOUCH_STANDARD_FINGER,
	TOUCH_PROXIMITY,
	TOUCH_STYLUS,
	TOUCH_HOVER,
	TOUCH_GLOVE,
	TOUCH_INVALID,
	NUM_OF_TOUCH_TYPES
} Touch_Type;

extern char* Touch_Type_Str[NUM_OF_TOUCH_TYPES];

typedef struct {
	uint8_t* data;
	size_t len;
	size_t index;
	size_t num_records;
	size_t max_len;
} ReportData;

typedef struct {
    long* data;
    uint read_len;
    uint max_len;
} FW_Self_Test_Results;

extern void output_debug_report(char* direction, char* format, char* label,
		char* type, ReportData* report);
extern void log_report_data(const ReportData* report, bool include_timestamp,
		char* label);
extern int write_report(const ReportData* report, const char* filepath);

#endif
