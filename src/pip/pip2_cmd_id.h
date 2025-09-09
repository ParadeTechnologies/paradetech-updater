/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_PIP_PIP2_CMD_ID_H_
#define PTLIB_PIP_PIP2_CMD_ID_H_

#include <stdint.h>

typedef enum {
	PIP2_CMD_ID_PING           = 0x00,
	PIP2_CMD_ID_STATUS         = 0x01,
	PIP2_CMD_ID_CTRL           = 0x02,
	PIP2_CMD_ID_CONFIG         = 0x03,
	PIP2_CMD_ID_CLEAR          = 0x05,
	PIP2_CMD_ID_RESET          = 0x06,
	PIP2_CMD_ID_VERSION        = 0x07,



	PIP2_CMD_ID_FILE_OPEN      = 0x10,
	PIP2_CMD_ID_FILE_CLOSE     = 0x11,
	PIP2_CMD_ID_FILE_READ      = 0x12,
	PIP2_CMD_ID_FILE_WRITE     = 0x13,
	PIP2_CMD_ID_FILE_IOCTL     = 0x14,
	PIP2_CMD_ID_FLASH_INFO     = 0x15,
	PIP2_CMD_ID_EXECUTE        = 0x16,
	PIP2_CMD_ID_GET_LAST_ERRNO = 0x17,
	PIP2_CMD_ID_EXIT_HOST_MODE = 0x18,
	PIP2_CMD_ID_READ_GPIO      = 0x19,


	PIP2_CMD_ID_EXECUTE_SCAN   = 0x21,










	PIP2_CMD_ID_SET_PARAMETER  = 0x40,
	PIP2_CMD_ID_GET_PARAMETER  = 0x41,
	PIP2_CMD_ID_SET_DDI_REG    = 0x42,
	PIP2_CMD_ID_GET_DDI_REG    = 0x43,















	PIP2_CMD_ID_NET_TRANS      = 0x70,





	PIP2_CMD_ID_EXTEND         = 0x7F,

	NUM_PIP2_CMD_IDS           = 128
} PIP2_Cmd_ID;

extern char* PIP2_CMD_NAMES[NUM_PIP2_CMD_IDS];

#endif
