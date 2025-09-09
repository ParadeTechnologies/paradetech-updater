/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef _PTLIB_FILE_H
#define _PTLIB_FILE_H

#include <regex.h>
#include "../ptstr_char.h"
#include "../logging.h"

typedef enum {
	POLL_STATUS_GOT_DATA,
	POLL_STATUS_TIMEOUT,
	POLL_STATUS_ERROR,
	POLL_STATUS_SKIP,
	NUM_OF_POLL_STATUSES
} Poll_Status;

extern int file_copy(char *copy_from_path, char *copy_to_path);
extern int file_insert(char *source_file_path, char *working_dir,
		char *regex_str,
		char *string_to_insert);
extern Poll_Status fpoll_inbound_data(FILE* fptr, time_t timeout);

#endif
