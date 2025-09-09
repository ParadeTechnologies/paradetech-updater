/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_HID_H_
#define PTLIB_HID_H_

#include <stdint.h>

#define HID_MAX_INPUT_REPORT_SIZE  0xFFFF
#define HID_MAX_OUTPUT_REPORT_SIZE 0xFFFF

typedef enum {
	HID_REPORT_ID_ANY                  = 0x00,
	HID_REPORT_ID_FINGER               = 0x01,
	HID_REPORT_ID_STYLUS               = 0x02,
	HID_REPORT_ID_FEATURE              = 0x03,
	HID_REPORT_ID_COMMAND              = 0x04,
	HID_REPORT_ID_VENDOR_FINGER        = 0x41,
	HID_REPORT_ID_VENDOR_STYLUS        = 0x42,
	HID_REPORT_ID_SOLICITED_RESPONSE   = 0x44,
	HID_REPORT_ID_UNSOLICITED_RESPONSE = 0x45
} HID_Report_ID;

typedef struct {
	uint16_t hid_desc_len;
	uint16_t bcd_version;
	uint16_t rpt_desc_len;
	uint16_t rpt_desc_register;
	uint16_t input_register;
	uint16_t max_input_len;
	uint16_t output_register;
	uint16_t max_output_len;
	uint16_t cmd_register;
	uint16_t data_register;
	uint16_t vendor_id;
	uint16_t product_id;
	uint16_t version_id;
	uint32_t reserved;
} __attribute__((packed)) HID_Descriptor;

typedef struct {
	uint8_t report_id;
	uint8_t payload_len_lsb;
	uint8_t payload_len_msb;
	struct {
		uint8_t seq                : 3;
		uint8_t tag                : 1;
		uint8_t more_data          : 1;
		uint8_t reserved_section_1 : 3;
	} __attribute__((packed));
	struct {
		uint8_t cmd_id : 7;
		uint8_t resp   : 1;
	} __attribute__((packed));
	uint8_t* cmd_specific_data;
} __attribute__((packed)) HID_Output_PIP3_Command;

#define HID_INPUT_REPORT_ID_BYTE_INDEX  0
#define HID_INPUT_PAYLOAD_LEN_LSB_INDEX 1
#define HID_INPUT_PAYLOAD_LEN_MSB_INDEX 2

typedef struct {
	uint8_t report_id;
	struct {
		uint8_t more_reports       : 1;
		uint8_t first_report       : 1;
		uint8_t reserved_section_1 : 6;
	} __attribute__((packed));
	uint8_t payload_len_lsb;
	uint8_t payload_len_msb;
	struct {
		uint8_t seq                : 3;
		uint8_t tag                : 1;
		uint8_t more_data          : 1;
		uint8_t reserved_section_2 : 3;
	} __attribute__((packed));
	struct {
		uint8_t cmd_id : 7;
		uint8_t resp   : 1;
	} __attribute__((packed));
	uint8_t* rsp_specific_data;
} __attribute__((packed)) HID_Input_PIP3_Response;

#endif
