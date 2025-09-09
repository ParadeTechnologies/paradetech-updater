/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef _PTSTR_CHAR_H
#define _PTSTR_CHAR_H

#include <ctype.h>
#include <stdint.h>
#include <string.h>

extern uint8_t hex_byte_to_int(char number);
extern void hex_str_to_uint(char *str, uint8_t *int_array, char* delimiter);
extern void int_to_hex_byte(unsigned char val, char *ascii);
extern size_t strlcpy(char *dst, const char *src, size_t siz);
extern size_t strlcat(char *dst, const char *src, size_t siz);
extern char* trim_whitespace(char *str);

#endif
