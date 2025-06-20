/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_PIP_PIP3_CMD_ID_H_
#define PTLIB_PIP_PIP3_CMD_ID_H_

#include <stdint.h>

typedef enum {
	PIP3_CMD_ID_PING                      = 0x00,
	PIP3_CMD_ID_STATUS                    = 0x01,
	PIP3_CMD_ID_CTRL                      = 0x02,
	PIP3_CMD_ID_CONFIG                    = 0x03,
	PIP3_CMD_ID_SWITCH_IMAGE              = 0x04,
	PIP3_CMD_ID_SWITCH_ACTIVE_PROCESSOR   = 0x05,
	PIP3_CMD_ID_RESET                     = 0x06,
	PIP3_CMD_ID_VERSION                   = 0x07,



	PIP3_CMD_ID_FILE_OPEN                 = 0x10,
	PIP3_CMD_ID_FILE_CLOSE                = 0x11,
	PIP3_CMD_ID_FILE_READ                 = 0x12,
	PIP3_CMD_ID_FILE_WRITE                = 0x13,
	PIP3_CMD_ID_FILE_IOCTL                = 0x14,
	PIP3_CMD_ID_FLASH_INFO                = 0x15,
	PIP3_CMD_ID_EXECUTE                   = 0x16,
	PIP3_CMD_ID_GET_LAST_ERRNO            = 0x17,
	PIP3_CMD_ID_EXIT_HOST_MODE            = 0x18,
	PIP3_CMD_ID_READ_GPIO                 = 0x19,


	PIP3_CMD_ID_VERIFY_DATA_BLOCK_CRC     = 0x20,
	PIP3_CMD_ID_GET_DATA_ROW_SIZE         = 0x21,
	PIP3_CMD_ID_GET_DATA_BLOCK            = 0x22,
	PIP3_CMD_ID_WRITE_DATA_BLOCK          = 0x23,
	PIP3_CMD_ID_GET_DATA_STRUCTURE        = 0x24,
	PIP3_CMD_ID_LOAD_SELF_TEST_PARAM      = 0x25,
	PIP3_CMD_ID_RUN_SELF_TEST             = 0x26,
	PIP3_CMD_ID_GET_SELF_TEST_RESULTS     = 0x27,
	PIP3_CMD_ID_INITIALIZE_BASELINE       = 0x29,
	PIP3_CMD_ID_EXECUTE_SCAN              = 0x2A,
	PIP3_CMD_ID_RETRIEVE_PANEL_SCAN       = 0x2B,
	PIP3_CMD_ID_START_SENSOR_DATA_MODE    = 0x2C,
	PIP3_CMD_ID_STOP_ASYNC_DEBUG_DATA     = 0x2D,
	PIP3_CMD_ID_START_TRACKING_HEATMAP    = 0x2E,
	PIP3_CMD_ID_CALIBRATE                 = 0x30,
	PIP3_CMD_ID_START_BOOTLOADER          = 0x31,
	PIP3_CMD_ID_GET_SYSINFO               = 0x32,
	PIP3_CMD_ID_SUSPEND_SCAN              = 0x33,
	PIP3_CMD_ID_RESUME_SCAN               = 0x34,
	PIP3_CMD_ID_GET_PARAM                 = 0x35,
	PIP3_CMD_ID_SET_PARAM                 = 0x36,
	PIP3_CMD_ID_GET_NOISE_METRICS         = 0x37,
	PIP3_CMD_ID_ENTER_EASYWAKE_STATE      = 0x39,
	PIP3_CMD_ID_SET_DBG_PARAMETER         = 0x3A,
	PIP3_CMD_ID_GET_DBG_PARAMETER         = 0x3B,
	PIP3_CMD_ID_SET_DDI_REG               = 0x3C,
	PIP3_CMD_ID_GET_DDI_REG               = 0x3D,
	PIP3_CMD_ID_REALTIME_SIGNAL_DATA_MODE = 0x3E,

	NUM_PIP3_CMD_IDS                  = 63
} PIP3_Cmd_ID;

extern char* PIP3_CMD_NAMES[NUM_PIP3_CMD_IDS];

#endif 
