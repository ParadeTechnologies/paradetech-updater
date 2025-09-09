/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_PIP_FW_BIN_HEADER_H_
#define PTLIB_PIP_FW_BIN_HEADER_H_

#include <stdint.h>

typedef struct {
	uint8_t header_len;
	uint8_t ttpid[2];
	uint8_t fw_major_version;
	uint8_t fw_minor_version;
	uint8_t fw_crc[4];
	uint8_t fw_rev_control[4];
	uint8_t silicon_id[2];
	uint8_t silicon_rev[2];
	uint8_t config_version[2];
    uint8_t encrypted_hex_file_len[4];
} __attribute__((packed)) FW_Bin_Header;

#define FW_BIN_HEADER_SIZE sizeof(FW_Bin_Header)

#endif
