/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "logging.h"

#define PREFIX_CSV_LEN 10

static int verbose_level = 1;
static FILE *fp_csv_file = NULL;
static FILE *fp_daemon_log_file = NULL;
static bool csv_debug_logging_enabled;

static bool kmsg_written = false;

static bool show_timestamp_fatal = false;
static bool show_timestamp_result = false;
static bool show_timestamp_error = false;
static bool show_timestamp_warning = false;
static bool show_timestamp_info = false;
static bool show_timestamp_debug = false;
static bool show_timestamp_ptc = false;
static bool show_timestamp_nolevel_noprefix = false;

void logging_fp_csv_file_set(FILE *file) {
	fp_csv_file = file;
}

void enable_csv_debug_logging_set(bool enable){
	csv_debug_logging_enabled = enable;
}

void logging_fp_csv_file_clear() {
	fp_csv_file = NULL;
}

void logging_fp_daemon_log_file_set(FILE *file) {
	fp_daemon_log_file = file;
}

void logging_fp_daemon_log_file_clear() {
	fp_daemon_log_file = NULL;
}

void kmsg_written_set() {
	kmsg_written = true;
}

void kmsg_written_clear() {
	kmsg_written = false;
}

bool get_kmsg_written() {
	return kmsg_written;
}

bool timestamp_level_get(enum verbose_levels level)
{
	switch (level) {
	case FATAL:
		return show_timestamp_fatal;
	case RESULT:
		return show_timestamp_result;
	case ERROR:
		return show_timestamp_error;
	case WARNING:
		return show_timestamp_warning;
	case INFO:
		return show_timestamp_info;
	case DEBUG:
		return show_timestamp_debug;
	case PTC:
		return show_timestamp_ptc;
	case NOLEVEL_NOPREFIX:
		return show_timestamp_nolevel_noprefix;
	default:
		output(FATAL, "Unrecognized log level enum value: %d\n", level);
		return false;
	}
}

void timestamp_level_set( 
		bool fatal, bool result, bool error, bool warning,
		bool info, bool debug, bool ptc, bool nolevel_noprefix)
{
	show_timestamp_fatal = fatal;
	show_timestamp_result = result;
	show_timestamp_error = error;
	show_timestamp_warning = warning;
	show_timestamp_info = info;
	show_timestamp_debug = debug;
	show_timestamp_ptc = ptc;
	show_timestamp_nolevel_noprefix = nolevel_noprefix;
}

void verbose_level_set(int new_level) {
	if (new_level >= DEBUG) {
		verbose_level = DEBUG;
	}
	else {
		verbose_level = new_level;
	}
}

int verbose_level_get() {
		return verbose_level;
}

void _output_stdout(int type, char *message)
{
	int prefix_stdout_len = 10;
	char prefix_stdout[prefix_stdout_len];
	bool print_stdout = true;
	FILE* print_fd = stdout;

	prefix_stdout[0] = '\0';

	if (type == QUIET) {
		print_stdout = false;
	} else if ((type == FATAL) && (verbose_level >= FATAL)) {
		strlcpy(prefix_stdout, "FATAL: ", prefix_stdout_len);
		print_fd = stderr;
	} else if (type == RESULT && (verbose_level >= RESULT)) {
		strlcpy(prefix_stdout, "RESULT: ", prefix_stdout_len);
	} else if (type == ERROR && (verbose_level >= ERROR)) {
		strlcpy(prefix_stdout, "ERROR:  ", prefix_stdout_len);
		print_fd = stderr;
	} else if (type == WARNING && (verbose_level >= WARNING)) {
		strlcpy(prefix_stdout, "WARNING: ", prefix_stdout_len);
	} else if (type == INFO && (verbose_level >= INFO)) {
		strlcpy(prefix_stdout, "INFO: ", prefix_stdout_len);
	} else if (type == PTC) {
		strlcpy(prefix_stdout, "PTC: ", prefix_stdout_len);
	} else if (type == NOLEVEL_NOPREFIX) {
		strlcpy(prefix_stdout, "", prefix_stdout_len);
	} else if (type >= DEBUG && (verbose_level >= DEBUG)) {
		strlcpy(prefix_stdout, "DEBUG:  ", prefix_stdout_len);
	} else {
		print_stdout = false;
	}
	if (print_stdout && timestamp_level_get(type)) {
		float uptime;
		struct timespec up_ts;

		clock_gettime(CLOCK_MONOTONIC, &up_ts);
		uptime = (float) up_ts.tv_sec + ((float) up_ts.tv_nsec / (float) 1e9);
		if (NULL == fp_daemon_log_file) {
			fprintf(print_fd, "[%13.6f] %s%s", uptime, prefix_stdout, message);
			fflush(print_fd);
		} else {
			fprintf(fp_daemon_log_file, "[%13.6f] %s%s", uptime, prefix_stdout,
					message);
			fflush(fp_daemon_log_file);
		}
	} else if (print_stdout) {
		if (NULL == fp_daemon_log_file) {
			fprintf(print_fd, "%s%s", prefix_stdout, message);
			fflush(print_fd);
		} else {
			fprintf(fp_daemon_log_file, "%s%s", prefix_stdout, message);
			fflush(fp_daemon_log_file);
		}
	}
}

void _output_csv(int type, char *message)
{
	char prefix_csv[PREFIX_CSV_LEN] = "";
	bool print_csv = true;

	switch (type) {
		case QUIET:
		case RESULT:
			print_csv = false;
			break;

		case FATAL:
			strlcpy(prefix_csv, ".FATAL,", PREFIX_CSV_LEN);
			break;

		case ERROR:
			strlcpy(prefix_csv, ".ERROR,", PREFIX_CSV_LEN);
			break;

		case WARNING:
			strlcpy(prefix_csv, ".WARNING,", PREFIX_CSV_LEN);
			break;
		case INFO:
			if (csv_debug_logging_enabled && verbose_level >= DEBUG) {
				strlcpy(prefix_csv, ".INFO,", PREFIX_CSV_LEN);
			} else {
				print_csv = false;
			}
			break;

		case DEBUG:
			if (csv_debug_logging_enabled && verbose_level >= DEBUG) {
				strlcpy(prefix_csv, ".DEBUG,", PREFIX_CSV_LEN);
			} else {
				print_csv = false;
			}
			break;

		default:
			print_csv = false;
			break;
	}

	if(print_csv) {
		struct sysinfo info;
		char *csv_str = NULL;
		sysinfo(&info);
		asprintf(&csv_str, "%s[%ld] %s", prefix_csv, info.uptime, message);
		fprintf(fp_csv_file, "%s", csv_str);
		free(csv_str);
	}
}

void _output_kmsg(int type, char *message)
{
	int prefix_kmsg_len = 10;
	char prefix_kmsg[prefix_kmsg_len];

	prefix_kmsg[0] = '\0';
	bool print_kmsg = true;

	if (type == QUIET) {
		print_kmsg = false;
	} else if (type == FATAL) {
		strlcpy(prefix_kmsg, "FATAL: ", prefix_kmsg_len);
	} else if (type == RESULT) {
		print_kmsg = false;
	} else if (type == ERROR) {
		strlcpy(prefix_kmsg, "ERROR: ", prefix_kmsg_len);
	} else if (type == WARNING) {
		strlcpy(prefix_kmsg, "WARNING: ", prefix_kmsg_len);
	} else if (type == INFO) {
		print_kmsg = false;
	} else if (type >= DEBUG) {
		print_kmsg = false;
	} else {
		print_kmsg = false;
	}

	if(print_kmsg) {
		static FILE *fp_kmsg_file = NULL;
		fp_kmsg_file = fopen("/dev/kmsg", "a");
		if(fp_kmsg_file != NULL) {
			fprintf(fp_kmsg_file, "PtMFG %s%s", prefix_kmsg, message);
			fflush(fp_kmsg_file);
			fclose(fp_kmsg_file);
		}
	}
}

void output(int type, const char *fmt,
		... 
		)
{
	int errno_backup = errno;
	va_list ap;
	char *output_str_no_prefix = NULL;
	va_start(ap, fmt);
	vasprintf(&output_str_no_prefix, fmt, ap);
	va_end(ap);

	if ((type == WARNING) || (type == ERROR) || (type == FATAL)) {
		kmsg_written_set();
	}

	_output_stdout(type, output_str_no_prefix);

	if (fp_csv_file != NULL ) { _output_csv(type, output_str_no_prefix); }
	_output_kmsg(type, output_str_no_prefix);

	free(output_str_no_prefix);
	errno = errno_backup;
}
