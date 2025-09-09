/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "ptstr_char.h"

size_t strlcpy(char *dst, const char *src, size_t dsize) {
	const char *osrc = src;
	size_t nleft = dsize;

	if (nleft != 0) {
		while (--nleft != 0) {
			if ((*dst++ = *src++) == '\0')
				break;
		}
	}

	if (nleft == 0) {
		if (dsize != 0)
			*dst = '\0';
		while (*src++)
			;
	}
	return(src - osrc - 1);
}

size_t strlcat(char *dst, const char *src, size_t siz) {
	char *d = dst;
	const char *s = src;
	size_t n = siz;
	size_t dlen;

	while (n-- != 0 && *d != '\0')
		d++;
	dlen = d - dst;
	n = siz - dlen;

	if (n == 0)
		return(dlen + strlen(s));
	while (*s != '\0') {
		if (n != 1) {
			*d++ = *s;
			n--;
		}
		s++;
	}
	*d = '\0';
	return(dlen + (s - src));
}

char *trim_whitespace(char *str) {
  char *end;

  while(isspace((unsigned char)*str)) str++;

  if(*str == 0) {
	  return str;
  }

  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)) end--;

  *(end+1) = 0;

  return str;
}

void int_to_hex_byte(unsigned char i, char *s)
{
	s += 2;
	*s = '\0';

	for (unsigned char n = 2; n != 0; --n) {
		*--s = "0123456789ABCDEF"[i & 0x0F];
		i >>= 4;
	}
}

uint8_t hex_byte_to_int(char number) {
	if (number >= '0' && number <= '9') return number - '0';
	else if (number >= 'a' && number <= 'f') return number - 'a' + 0x0a;
	else if (number >= 'A' && number <= 'F') return number - 'A' + 0X0a;
	else return 0;
}

void hex_str_to_uint(char *str, uint8_t *int_array, char* delimiter) {
	int str_len = strlen(str);
	int delimiter_len = strlen(delimiter);
	int byte_count = 0;

	for (int i = 0; i < str_len; i += (2 + delimiter_len)) {
		int_array[byte_count] = (hex_byte_to_int(str[i]) << 4) +
				hex_byte_to_int(str[i+1]);
		byte_count++;
	}
}

