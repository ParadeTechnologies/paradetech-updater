/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "hidraw.h"

#define HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN 20
#define REPORT_BUFFER_SIZE 256 

static char hidraw_sysfs_node_file[HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN] =
		HIDRAW0_SYSFS_NODE_FILE;
static int hidraw0_fd;
static bool hidraw0_open = false;

static int self_pipe_fd[2];
enum { SELF_PIPE_READ, SELF_PIPE_WRITE };
static bool stop_reading;

static pthread_t   report_reader_tid;

typedef enum {
	REPORT_READER_THREAD_STATUS_ACTIVE,
	REPORT_READER_THREAD_STATUS_NOT_STARTED,
	REPORT_READER_THREAD_STATUS_EXIT
} Report_Reader_Thread_Status;

static Report_Reader_Thread_Status report_reader_thread_status;
static Poll_Status                 report_read_status;

typedef struct {
	ReportData report;
	bool ready;
} Buffer_Entry;

static pthread_mutex_t report_buffer_mutex;
static Buffer_Entry    report_buffer[REPORT_BUFFER_SIZE] = {NULL};

static uint            report_buffer_least_recent_index;
static uint            report_buffer_count;

static HID_Descriptor _hid_desc;
static bool hid_desc_read = false;
static size_t hid_output_report_size;
static size_t hid_input_report_size;

static Poll_Status _consume_report(HID_Report_ID target_report_id,
		uint* next_victim_report_index, bool* more_reports);
static int _get_max_input_len();
static int _get_max_output_len();
static Poll_Status _read_report(ReportData* report);
static void* _report_reader_thread(void* arg);
static int _try_hidraw_sysfs_node(char* sysfs_node_file, int vendor_id,
		int product_id);

Channel hidraw_channel = {
	.type               = CHANNEL_TYPE_HIDRAW,
	.setup              = start_hidraw_report_reader,
	.get_hid_descriptor = get_hid_descriptor_from_hidraw,
	.send_report        = send_report_via_hidraw,
	.get_report         = get_report_from_hidraw,
	.teardown           = stop_hidraw_report_reader,
};

int auto_detect_hidraw_sysfs_node(int vendor_id, int product_id)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (vendor_id > 0xFFFF) {
		output(ERROR,
				"%s: Invalid vendor ID. It must be less than 0xFFFF but got "
				"0x%X.\n", __func__, vendor_id);
		return EXIT_FAILURE;
	} else if (product_id > 0xFFFF) {
		output(ERROR,
				"%s: Invalid product ID. It must be less than 0xFFFF but got "
				"0x%X.\n", __func__, product_id);
		return EXIT_FAILURE;
	}

	DIR* dev_dir;
	bool dev_dir_opened = false;
	regex_t regex;
	bool sysfs_node_found = false;

	if (regcomp(&regex, "hidraw[[:digit:]+]$", 0)) {
		output(ERROR,
				"%s: Failed to compile the regex pattern.\n", __func__);
		return EXIT_FAILURE;
	}

	dev_dir = opendir("/dev/");
	if (dev_dir == NULL) {
		output(ERROR, "%s: Failed to open the /dev/ directory. %s [%d]\n",
				__func__,  strerror(errno), errno);
		goto RETURN;
	}
	dev_dir_opened = true;

	while (!sysfs_node_found) {
		const struct dirent* dev_dir_entry = readdir(dev_dir);
		if (dev_dir_entry == NULL) {
			output(ERROR,
					"%s: Failed to read from the /dev/ directory. %s [%d]\n",
					__func__,  strerror(errno), errno);
			goto RETURN;
		}

		if (regexec(&regex, dev_dir_entry->d_name, 0, NULL, 0) == 0) {
			char sysfs_node_file[HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN];
			snprintf(sysfs_node_file, HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN,
					"/dev/%s", dev_dir_entry->d_name);

			output(INFO, "Trying HIDRAW sysfs node = '%s'\n", sysfs_node_file);

			if (EXIT_SUCCESS == _try_hidraw_sysfs_node(sysfs_node_file,
													vendor_id, product_id)) {
				sysfs_node_found = true;
			}
		}
	}

RETURN:
	if (dev_dir_opened) {
		(void) closedir(dev_dir);
	}

	regfree(&regex);

	return sysfs_node_found ? EXIT_SUCCESS : EXIT_FAILURE;
}

void clear_hidraw_report_buffer()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	pthread_mutex_lock(&report_buffer_mutex);
	for (int i = 0; i < REPORT_BUFFER_SIZE; i++) {
		report_buffer[i].report.len = 0;
		report_buffer[i].ready = false;
	}
	report_buffer_least_recent_index = 0;
	report_buffer_count = 0;
	pthread_mutex_unlock(&report_buffer_mutex);
}

int get_hid_descriptor_from_hidraw(HID_Descriptor* hid_desc)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int fd;
	bool file_opened = false;
	int max_input_len;
	int max_output_len;
	int rc = EXIT_FAILURE;
	int read_rc;
	int rpt_desc_size;
	struct hidraw_devinfo dev_info;

	if (hid_desc == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	}

	if (hid_desc_read) {
		output(DEBUG, "HID descriptor already read.\n");
		goto COPY;
	}

	fd = open(hidraw_sysfs_node_file, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		output(ERROR, "%s: Failed to open %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		return EXIT_FAILURE;
	}
	file_opened = true;

	read_rc = ioctl(fd, HIDIOCGRDESCSIZE, &rpt_desc_size);
	if (read_rc < 0) {
		output(ERROR,
				"%s: Failed to read the Report Descriptor size from %s. "
				"%s [%d]\n",
				__func__, hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	max_input_len = _get_max_input_len();
	if (max_input_len <= 0) {
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	max_output_len = _get_max_output_len();
	if (max_output_len <= 0) {
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	read_rc = ioctl(fd, HIDIOCGRAWINFO, &dev_info);
	if (read_rc < 0) {
		output(ERROR,
				"%s: Failed to read the raw device info from %s. "
				"%s [%d]\n",
				__func__, hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	_hid_desc.hid_desc_len      = 0x001E;
	_hid_desc.bcd_version       = 0x0100;
	_hid_desc.rpt_desc_len      = (uint16_t) rpt_desc_size;
	_hid_desc.rpt_desc_register = 0x0002;
	_hid_desc.input_register    = 0x0003;
	_hid_desc.max_input_len     = (uint16_t) max_input_len;
	_hid_desc.output_register   = 0x0004;
	_hid_desc.max_output_len    = (uint16_t) max_output_len;
	_hid_desc.cmd_register      = 0x0004;
	_hid_desc.data_register     = 0x0005;
	_hid_desc.vendor_id         = dev_info.vendor;
	_hid_desc.product_id        = dev_info.product;
	_hid_desc.version_id        = 0x0000;

COPY:
	memcpy((void*) hid_desc, (void*) &_hid_desc, sizeof(_hid_desc));
	hid_desc_read = true;
	rc = EXIT_SUCCESS;

RETURN:
	if (file_opened) {
		close(fd);
	}
	return rc;
}

Poll_Status get_report_from_hidraw(ReportData* report, bool apply_timeout,
		long double timeout_val)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	Poll_Status rc;
	struct timeval start_time;

	if (report == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return POLL_STATUS_ERROR;
	}

	switch (report_reader_thread_status) {
	case REPORT_READER_THREAD_STATUS_NOT_STARTED:
		output(ERROR, "%s: Report reader thread has not been started.\n",
				__func__);
		return POLL_STATUS_ERROR;
	case REPORT_READER_THREAD_STATUS_EXIT:
		if (report_buffer_count == 0) {
			output(DEBUG, "Report reader thread has already terminated. "
					"No more reports to read.\n");
			return POLL_STATUS_SKIP;
		} else {
			goto GET_REPORT;
		}
	default: 
		;
	}

	gettimeofday(&start_time, 0);
	while (report_buffer_count == 0
			|| !report_buffer[report_buffer_least_recent_index].ready) {
		if (report_reader_thread_status != REPORT_READER_THREAD_STATUS_ACTIVE) {
			return report_read_status;
		} else if (apply_timeout
				&& time_limit_reached(&start_time, timeout_val)) {
			return POLL_STATUS_TIMEOUT;
		}
	}

GET_REPORT:
	pthread_mutex_lock(&report_buffer_mutex);

	memcpy((void*) report->data,
			(void*) report_buffer[report_buffer_least_recent_index].report.data,
			report_buffer[report_buffer_least_recent_index].report.len);
	report->len = report_buffer[report_buffer_least_recent_index].report.len;
	report_buffer[report_buffer_least_recent_index].report.len = 0;
	report_buffer[report_buffer_least_recent_index].ready = false;

	rc = report_read_status;

	report_buffer_least_recent_index = ((report_buffer_least_recent_index + 1)
			% REPORT_BUFFER_SIZE);
	report_buffer_count--;

	pthread_mutex_unlock(&report_buffer_mutex);

	return rc;
}

int get_report_descriptor_from_hidraw(ReportData* rpt_desc)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	int read_rc;
	struct hidraw_report_descriptor _rpt_desc;
	int fd;

	memset(&_rpt_desc, 0, sizeof(_rpt_desc));

	if (rpt_desc == NULL || rpt_desc->data == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	}

	fd = open(hidraw_sysfs_node_file, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		output(ERROR, "%s: Failed to open %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		return EXIT_FAILURE;
	}

	read_rc = ioctl(fd, HIDIOCGRDESCSIZE, &_rpt_desc.size);
	if (read_rc < 0) {
		output(ERROR,
				"%s: Failed to read the Report Descriptor size from %s. "
				"%s [%d]\n",
				__func__, hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	rpt_desc->len = _rpt_desc.size;
	read_rc = ioctl(fd, HIDIOCGRDESC, &_rpt_desc);
	if (read_rc < 0) {
		output(ERROR,
				"%s: Failed to read the Report Descriptor from %s. %s [%d]\n",
				__func__, hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	memcpy((void*) rpt_desc->data, (void*) _rpt_desc.value, _rpt_desc.size);

	rc = EXIT_SUCCESS;

RETURN:
	close(fd);
	return rc;
}

int init_hidraw_api(const char* sysfs_node_file, const HID_Descriptor* hid_desc)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	output(INFO, "Provided HIDRAW sysfs node: %s.\n", sysfs_node_file);

	if (strlen(sysfs_node_file) > HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN) {
		output(ERROR, "%s: The provided HIDRAW sysfs node file is %lu chars, "
			"but the max supported length is %lu chars.", __func__,
			strlen(sysfs_node_file), HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN);
		return EXIT_FAILURE;
	}

	strncpy(hidraw_sysfs_node_file, sysfs_node_file, strlen(sysfs_node_file));

	if (hid_desc != NULL) {
		output(DEBUG, "HID descriptor initialized prior to use of HIDRAW.\n");
		memcpy((void*) &_hid_desc, (void*) hid_desc, sizeof(_hid_desc));
		hid_desc_read = true;
	}

	return EXIT_SUCCESS;
}

int init_input_report(ReportData* report)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (report == NULL) {
		output(ERROR, "%s: Provided 'report' argument is NULL.\n", __func__);
		return EXIT_FAILURE;
	} else if (report->data != NULL) {
		output(ERROR, "%s: 'report->data' argument must be given as NULL.\n",
				__func__);
		return EXIT_FAILURE;
	}

	report->max_len = hid_input_report_size;
	report->data = (uint8_t*) malloc(report->max_len * sizeof(uint8_t));
	if (report->data == NULL) {
		output(ERROR, "%s: Memory allocation failed.\n", __func__);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

int send_report_via_hidraw(const ReportData* report)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	return write_report(report, hidraw_sysfs_node_file);
}

int start_hidraw_report_reader(HID_Report_ID report_id)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	HID_Descriptor hid_desc;

	report_reader_thread_status = REPORT_READER_THREAD_STATUS_NOT_STARTED;
	report_buffer_least_recent_index = 0;

	if (EXIT_SUCCESS != get_hid_descriptor_from_hidraw(&hid_desc)) {
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	hid_output_report_size = hid_desc.max_output_len - 2;
	hid_input_report_size = hid_desc.max_input_len - 2;

	hidraw0_fd = open(hidraw_sysfs_node_file, O_RDWR | O_NONBLOCK);
	if (hidraw0_fd < 0) {
		output(ERROR, "%s: Failed to open %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}
	hidraw0_open = true;

	if (pipe(self_pipe_fd) < 0) {
		output(ERROR,
				"%s: Failed to open pipe for communicating with report reader "
				"thread. %s [%d]\n", __func__, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	stop_reading = false;

	for (int i = 0; i < REPORT_BUFFER_SIZE; i++) {
		report_buffer[i].report.max_len = hid_input_report_size;
		report_buffer[i].report.len = 0;
		report_buffer[i].ready = false;
		report_buffer[i].report.data = malloc(report_buffer[i].report.max_len);
		if (NULL == report_buffer[i].report.data) {
			output(ERROR, "%s: Memory allocation failed.\n", __func__);
			rc = EXIT_FAILURE;
			goto RETURN;
		}
	}

	if (0 != pthread_create(&report_reader_tid, NULL, _report_reader_thread,
			(void*) &report_id)) { 
		output(ERROR,
				"%s: Failed to start thread for reading reports from %s. "
				"%s [%d]\n",
				__func__, hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	while (report_reader_thread_status
			== REPORT_READER_THREAD_STATUS_NOT_STARTED);

	rc = EXIT_SUCCESS;

RETURN:
	if (rc != EXIT_SUCCESS) {
		stop_hidraw_report_reader();
	}

	return EXIT_SUCCESS;
}

int stop_hidraw_report_reader()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	stop_reading = true;

	char stop_signal[] = "S";
	write(self_pipe_fd[SELF_PIPE_WRITE], stop_signal, 2);

	struct timeval start_time;
	gettimeofday(&start_time, 0);
	while (!time_limit_reached(&start_time, 5)
			&& pthread_kill(report_reader_tid, 0) != ESRCH);

	if (pthread_kill(report_reader_tid, 0) != ESRCH) {
		output(DEBUG, "Terminating the thread reading HIDRAW reports.\n");
		pthread_kill(report_reader_tid, SIGINT);
		sleep_ms(1000);
		if (pthread_kill(report_reader_tid, 0) != ESRCH) {
			output(WARNING,
					"The thread reading HIDRAW reports coundn't be interrupted."
					" Now taking a more forceful approach.\n");
			pthread_kill(report_reader_tid, SIGKILL);
		}
	} else {
		output(DEBUG,
				"The thread reading HIDRAW reports has already terminated.\n");
	}

	pthread_join(report_reader_tid, NULL);

	for (int i = 0; i < REPORT_BUFFER_SIZE; i++) {
		free(report_buffer[i].report.data);
		report_buffer[i].report.data = NULL;
		report_buffer[i].ready = false;
	}

	close(hidraw0_fd);
	hidraw0_open = false;
	close(self_pipe_fd[SELF_PIPE_READ]);
	close(self_pipe_fd[SELF_PIPE_WRITE]);

	return EXIT_SUCCESS;
}

static Poll_Status _consume_report(HID_Report_ID target_report_id,
		uint* next_victim_report_index, bool* more_reports)
{
	Poll_Status read_status = _read_report(
			&report_buffer[*next_victim_report_index].report);
	if (read_status != POLL_STATUS_GOT_DATA) {
		return read_status;
	}

	pthread_mutex_lock(&report_buffer_mutex);

	uint8_t report_id = report_buffer[*next_victim_report_index].report.data[
												HID_INPUT_REPORT_ID_BYTE_INDEX];
	read_status = (
			(target_report_id == HID_REPORT_ID_ANY
					|| target_report_id == report_id)
					? POLL_STATUS_GOT_DATA : POLL_STATUS_SKIP);

	if (read_status == POLL_STATUS_GOT_DATA) {
		const HID_Input_PIP3_Response* input_report = (
				(HID_Input_PIP3_Response*)
				report_buffer[*next_victim_report_index].report.data);

		*more_reports = (
			(
				   input_report->report_id == HID_REPORT_ID_SOLICITED_RESPONSE
				|| input_report->report_id == HID_REPORT_ID_UNSOLICITED_RESPONSE
			) ? input_report->more_reports : false
		);
		report_buffer[*next_victim_report_index].ready = true;

		*next_victim_report_index = (
				(*next_victim_report_index + 1) % REPORT_BUFFER_SIZE);
		report_buffer_count++;
		if (*next_victim_report_index
				== report_buffer_least_recent_index) {
			report_buffer_count--;
			report_buffer_least_recent_index = (
					(report_buffer_least_recent_index + 1)
					% REPORT_BUFFER_SIZE);
		}
	}

	pthread_mutex_unlock(&report_buffer_mutex);

	return read_status;
}

#define AVG_DELAY_BETWEEN_CMD_AND_RSP 5 

static int _get_max_input_len()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int fd;
	int max_input_len;

	fd = open(hidraw_sysfs_node_file, O_RDWR);
	if (fd < 0) {
		output(ERROR, "%s: Failed to open %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		return -1;
	}

	uint8_t suspend_scan_cmd_data[] = {
			0x04, 0x06, 0x00, 0x08, 0x33, 0x2C, 0xC0};
	ReportData suspend_scan_cmd = {
			.data = suspend_scan_cmd_data,
			.len  = sizeof(suspend_scan_cmd_data)
	};
	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_HID,
			"SUSPEND_SCAN", REPORT_TYPE_COMMAND, &suspend_scan_cmd);
	if (EXIT_SUCCESS
			!= write_report(&suspend_scan_cmd, hidraw_sysfs_node_file)) {
		max_input_len = -1;
		goto RETURN;
	}

	sleep_ms(AVG_DELAY_BETWEEN_CMD_AND_RSP);

	uint8_t ping_cmd_data[] = {0x04, 0x06, 0x00, 0x08, 0x00, 0x2A, 0xF0};
	ReportData ping_cmd = {
			.data = ping_cmd_data,
			.len  = sizeof(ping_cmd_data)
	};
	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_HID,
			"PING", REPORT_TYPE_COMMAND, &ping_cmd);
	if (EXIT_SUCCESS != write_report(&ping_cmd, hidraw_sysfs_node_file)) {
		max_input_len = -1;
		goto RETURN;
	}

	sleep_ms(AVG_DELAY_BETWEEN_CMD_AND_RSP);

	uint8_t ping_rsp[HID_MAX_INPUT_REPORT_SIZE];
	max_input_len = read(fd, ping_rsp, HID_MAX_INPUT_REPORT_SIZE);
	if (max_input_len < 0) {
		output(ERROR, "%s: Failed to read from %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
	} else if (max_input_len == 0) {
		output(ERROR, "%s: Zero bytes read from %s. Something went wrong.\n",
				__func__);
		max_input_len = -1;
	} else {
		max_input_len += 2;
		output(DEBUG, "Max HID input report length: %d bytes.\n",
				max_input_len);
	}

RETURN:
	return max_input_len;
}

static int _get_max_output_len()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	int fd = open(hidraw_sysfs_node_file, O_RDWR);
	if (fd < 0) {
		output(ERROR, "%s: Failed to open %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		return -1;
	}

	int max_output_len;
	uint8_t ping_cmd[HID_MAX_OUTPUT_REPORT_SIZE] = {
			0x04, 0x06, 0x00, 0x08, 0x00, 0x2A, 0xF0
	};
	int lower_len = 0;
	int upper_len = sizeof(ping_cmd);
	while (lower_len <= upper_len) {
		int mid_len = (upper_len + lower_len) / 2;
		int res = write(fd, ping_cmd, mid_len);
		if (res < 0) {
			upper_len = mid_len - 1;
		} else {
			max_output_len = res;
			lower_len = max_output_len + 1;
		}
		sleep_ms(AVG_DELAY_BETWEEN_CMD_AND_RSP);
	}

	max_output_len += 2;
	output(DEBUG, "Max HID output report length: %d bytes.\n", max_output_len);

	return max_output_len;
}

static Poll_Status _read_report(ReportData* report)
{
	int read_rc;
	fd_set read_set;
	int select_rc;

	FD_ZERO(&read_set);
	FD_SET(hidraw0_fd, &read_set);
	FD_SET(self_pipe_fd[SELF_PIPE_READ], &read_set);

	select_rc = select(self_pipe_fd[SELF_PIPE_READ] + 1, &read_set, NULL, NULL,
			NULL);
	if (select_rc == -1) {
		output(ERROR, "%s: A problem occurred while trying to read from %s. "
				"%s [%d]\n", __func__, hidraw_sysfs_node_file, strerror(errno),
				errno);
		return POLL_STATUS_ERROR;
	} else if (select_rc == 0) {
		output(DEBUG, "Polling timed-out for incoming data.\n");
		return POLL_STATUS_TIMEOUT;
	}

	if (stop_reading) {
		output(DEBUG, "Got signal to stop report reader thread.\n");
		return POLL_STATUS_TIMEOUT;
	}

	read_rc = read(hidraw0_fd, report->data, report->max_len);
	if (read_rc < 0) {
		output(ERROR, "%s: Failed to read from %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		return POLL_STATUS_ERROR;
	}
	report->len = read_rc;
	return POLL_STATUS_GOT_DATA;
}

static void* _report_reader_thread(void* arg)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	HID_Report_ID report_id = *((HID_Report_ID*) arg);
	uint next_victim_report_index = 0;
	Poll_Status read_status = POLL_STATUS_GOT_DATA;
	bool more_reports = false;

	report_reader_thread_status = REPORT_READER_THREAD_STATUS_ACTIVE;

	do {
		memset(report_buffer[next_victim_report_index].report.data, 0,
				report_buffer[next_victim_report_index].report.max_len);

		read_status = _consume_report(report_id, &next_victim_report_index,
				&more_reports);
	} while ((read_status == POLL_STATUS_GOT_DATA
			|| read_status == POLL_STATUS_SKIP)
			&& report_reader_thread_status
				== REPORT_READER_THREAD_STATUS_ACTIVE);

	pthread_mutex_lock(&report_buffer_mutex);
	report_read_status = (
			(read_status != POLL_STATUS_ERROR && report_buffer_count > 0)
			? POLL_STATUS_GOT_DATA : read_status);
	report_reader_thread_status = REPORT_READER_THREAD_STATUS_EXIT;
	pthread_mutex_unlock(&report_buffer_mutex);

	output(DEBUG, "%s: Leaving.\n", __func__);
	pthread_exit(NULL);
}

static int _try_hidraw_sysfs_node(char* sysfs_node_file, int vendor_id,
		int product_id)
{
	int rc = EXIT_FAILURE;
	struct hidraw_devinfo dev_info;

	int fd = open(sysfs_node_file, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		output(ERROR, "%s: Failed to open %s. %s [%d]\n", __func__,
				sysfs_node_file, strerror(errno), errno);
		return EXIT_FAILURE;
	}

	if (ioctl(fd, HIDIOCGRAWINFO, &dev_info) < 0) {
		output(ERROR,
				"%s: Failed to read the raw device info from %s. "
				"%s [%d]\n",
				__func__, hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	output(INFO, "Detected device info for %s VID = 0x%04X, PID = 0x%04X\n",
			sysfs_node_file, dev_info.vendor, dev_info.product);

	if (vendor_id == dev_info.vendor && product_id == dev_info.product) {
		rc = init_hidraw_api(sysfs_node_file, NULL);
	}

RETURN:
	close(fd);
	return rc;
}
