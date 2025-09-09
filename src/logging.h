/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

 
#ifndef _LOGGING_H
#define _LOGGING_H

#define _GNU_SOURCE

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sys/statvfs.h>
#include <sys/sysinfo.h>
#include <errno.h>
#include <dirent.h>
#include <time.h>
#include "ptstr_char.h"

enum verbose_levels {
	QUIET =    0,
	FATAL =    1,
	RESULT =   2,
	ERROR =    3,
	WARNING =  4,
	INFO =     5,
	DEBUG =    6,
	PTC = 98,
	NOLEVEL_NOPREFIX = 99
};

extern void display_result(int retry_number, const char test_name[],
		 int test_result, float durration, char test_details[],
		 const char *extra_info_str);
extern bool get_kmsg_written();
extern void kmsg_written_clear();
extern void kmsg_written_set();
extern void logging_fp_csv_file_clear();
extern void logging_fp_csv_file_set(FILE *file);
extern void enable_csv_debug_logging_set(bool enable);
extern void logging_fp_daemon_log_file_clear();
extern void logging_fp_daemon_log_file_set(FILE *file);
extern void output(int type, const char *fmt, ...);
extern bool timestamp_level_get(enum verbose_levels level);
extern void timestamp_level_set(bool fatal, bool result, bool error,
		bool warning, bool info, bool debug, bool ptc, bool nolevel_noprefix);
extern int verbose_level_get();
extern void verbose_level_set(int new_level);

#endif 
