/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef _CRC16_CCIT_H
#define _CRC16_CCIT_H

#include <stdio.h>
#include <stdint.h>

extern uint16_t calculate_crc16_ccitt(uint16_t seed, uint8_t *data,
		size_t length);

#endif
