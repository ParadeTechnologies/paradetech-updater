/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "hidraw.h"

#define HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN 20
#define REPORT_BUFFER_SIZE 256
#define I2C_DEV_BASE_PATH "/sys/bus/i2c/devices"
#define I2C_DEV_PATH_MAX 1024
#define I2C_DEV_NAME_MAX 64
#define DELAY_AFTER_DRIVER_UNBIND 100
#define DELAY_AFTER_DRIVER_BIND   300

#define AVG_DELAY_BETWEEN_CMD_AND_RSP 20

static char hidraw_sysfs_node_file[HIDRAW_SYSFS_NODE_FILE_MAX_STRLEN] =
		HIDRAW0_SYSFS_NODE_FILE;
static int hidraw0_fd;

static bool hidraw0_open = false;

static bool self_pipe_create = false;

static bool report_reader_create = false;

static bool report_buffer_create = false;

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
static Poll_Status _read_report(ReportData* report);
static void* _report_reader_thread(void* arg);
static int _try_hidraw_sysfs_node(char* sysfs_node_file, int vendor_id,
		int product_id);
static int _get_hid_descriptor_from_data_block(HID_Descriptor* hid_desc);
static int _pip_cmd_rsp(ReportData* report, ReportData* rsp);

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


static bool _is_target_hid_device(const char *i2c_dev_name, int vendor_id,
								  int product_id)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	char i2c_dev_path[I2C_DEV_PATH_MAX] = {0};
	DIR *hid_name_dir;
	regex_t regex;
	bool hid_dir_opened = false;
	bool hid_name_found = false;

	if (i2c_dev_name == NULL) {
		output(ERROR, "%s: Invalid pointer of i2c_dev_path.\n", __func__);
		return hid_name_found;
	}

	snprintf(i2c_dev_path, I2C_DEV_PATH_MAX,
			"%s/%s/", I2C_DEV_BASE_PATH, i2c_dev_name);
	output(DEBUG, "i2c_dev_path: %s\n", i2c_dev_path);

	if (regcomp(&regex, "^[0-9A-F]{4}:[0-9A-F]{4}:[0-9A-F]{4}\\.[0-9]{4}$",
				REG_EXTENDED | REG_NOSUB)) {
		output(ERROR, "%s: Failed to compile the regex pattern.\n", __func__);
		return hid_name_found;
	}

	hid_name_dir = opendir(i2c_dev_path);
	if (hid_name_dir == NULL) {
		output(ERROR, "%s: Failed to open the %s directory. %s [%d]\n",
			   __func__, hid_name_dir, strerror(errno), errno);
		goto RETURN;
	}
	hid_dir_opened = true;

	while (!hid_name_found) {
		const struct dirent *dir_entry = readdir(hid_name_dir);
		if (dir_entry == NULL) {
			output(INFO, "%s: Failed to read from the %s directory. %s [%d]\n",
				   __func__, i2c_dev_path, strerror(errno), errno);
			goto RETURN;
		}

		if (regexec(&regex, dir_entry->d_name, 0, NULL, 0) == 0) {
			unsigned int vid, pid;
			if (sscanf(dir_entry->d_name, "%*4x:%4x:%4x", &vid, &pid) == 2) {
				output(INFO, "Extracted VID: %04X, PID: %04X\n", vid, pid);
				if (vid == vendor_id && pid == product_id) {
					hid_name_found = true;
					goto RETURN;
				}
			} else {
				output(INFO, "%s: Failed to parse VID and PID from %s.\n",
					   __func__, dir_entry->d_name);
			}
		}
	}

RETURN:
	if (hid_dir_opened) {
		(void)closedir(hid_name_dir);
	}

	regfree(&regex);

	return hid_name_found;
}

static int _resolve_driver_link(const char *i2c_dev_name, char *driver_link,
								char *driver_path)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (i2c_dev_name == NULL || driver_link == NULL || driver_path == NULL) {
		output(ERROR,
			   "%s: Invalid pointer of i2c_dev_name, driver_link or "
			   "driver_path.\n",
			   __func__);
		goto FAIL;
	}

	char original_cwd[I2C_DEV_PATH_MAX];
	char cur_path[I2C_DEV_PATH_MAX];

	if (getcwd(original_cwd, sizeof(original_cwd)) == NULL) {
		output(ERROR, "%s: Failed to get current working directory. %s [%d]\n",
			   __func__, strerror(errno), errno);
		goto FAIL;
	}

	snprintf(cur_path, I2C_DEV_PATH_MAX, "%s/%s", I2C_DEV_BASE_PATH,
			 i2c_dev_name);
	if (chdir(cur_path) != 0) {
		output(ERROR, "%s: Failed to change directory to %s. %s [%d]\n",
			   __func__, cur_path, strerror(errno), errno);
		goto FAIL;
	}

	if (chdir(cur_path) != 0) {
		output(ERROR, "%s: Failed to change directory to %s. %s [%d]\n",
			   __func__, cur_path, strerror(errno), errno);
		goto FAIL;
	}

	if (realpath(driver_link, driver_path) == NULL) {
		output(ERROR, "%s: Failed to resolve the driver link %s. %s [%d]\n",
			   __func__, driver_link, strerror(errno), errno);
		goto FAIL;
	}

	if (chdir(original_cwd) != 0) {
		output(ERROR,
			   "%s: Failed to change back to original directory %s. %s [%d]\n",
			   __func__, original_cwd, strerror(errno), errno);
		goto FAIL;
	}

	return EXIT_SUCCESS;

FAIL:
	return EXIT_FAILURE;
}

static int _try_find_driver_path(const char *i2c_dev_name, char *driver_path)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (i2c_dev_name == NULL || driver_path == NULL) {
		output(ERROR, "%s: Invalid pointer of i2c_dev_name or driver_link.\n",
			   __func__);
		return EXIT_FAILURE;
	}

	char driver_path_file[I2C_DEV_PATH_MAX] = {0};
	char driver_link[I2C_DEV_PATH_MAX] = {0};
	snprintf(driver_path_file, I2C_DEV_PATH_MAX, "%s/%s/driver",
			 I2C_DEV_BASE_PATH, i2c_dev_name);

	ssize_t len =
		readlink(driver_path_file, driver_link, I2C_DEV_PATH_MAX - 1);
	if (len < 0) {
		output(DEBUG, "%s: Failed to read the driver link %s. %s [%d]\n",
			   __func__, driver_path_file, strerror(errno), errno);
		return EXIT_FAILURE;
	} else if (len >= I2C_DEV_PATH_MAX) {
		output(ERROR, "%s: Driver link path is too long.\n", __func__);
		return EXIT_FAILURE;
	}
	driver_link[len] = '\0';
	output(DEBUG, "device_name: %s, driver_link: %s\n",
		i2c_dev_name, driver_link);

	return _resolve_driver_link(i2c_dev_name, driver_link, driver_path);
}

static int auto_detect_hid_dut(int vid, int pid, char *i2c_dev_name, char *driver_path)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (i2c_dev_name == NULL || driver_path == NULL) {
		output(ERROR, "%s: Invalid pointer of i2c_dev_path or driver_path.\n",
			   __func__);
		return EXIT_FAILURE;
	}

	char tmp_driver_path[I2C_DEV_PATH_MAX] = {0};
	DIR *dev_dir;
	bool dev_dir_opened = false;
	bool hid_dut_found = false;

	dev_dir = opendir(I2C_DEV_BASE_PATH);
	if (dev_dir == NULL) {
		output(ERROR, "%s: Failed to open the %s directory. %s [%d]\n",
			   __func__, I2C_DEV_BASE_PATH, strerror(errno), errno);
		goto RETURN;
	}
	dev_dir_opened = true;

	while (!hid_dut_found) {
		const struct dirent *dev_dir_entry = readdir(dev_dir);
		if (dev_dir_entry == NULL) {
			output(ERROR,
				   "%s: Failed to read from the /dev/ directory. %s [%d]\n",
				   __func__, strerror(errno), errno);
			goto RETURN;
		}

		if (dev_dir_entry->d_name[0] == '.') {
			continue;
		}

		if (EXIT_SUCCESS !=
			_try_find_driver_path(dev_dir_entry->d_name, tmp_driver_path)) {
			output(DEBUG, "%s: Failed to find driver path for %s.\n", __func__,
				   dev_dir_entry->d_name);
			continue;
		}

		if (_is_target_hid_device(dev_dir_entry->d_name, vid, pid)) {
			strlcpy(i2c_dev_name, dev_dir_entry->d_name, I2C_DEV_NAME_MAX);
			strlcpy(driver_path, tmp_driver_path, I2C_DEV_PATH_MAX);
			hid_dut_found = true;
			goto RETURN;
		}
	}

RETURN:
	if (hid_dut_found) {
		output(DEBUG, "Detected HID device name: %s\n", i2c_dev_name);
		output(DEBUG, "Detected HID driver path: %s\n", driver_path);
	} else {
		output(ERROR,
			   "%s: No HID device found with VID: 0x%04X and PID: 0x%04X.\n",
			   __func__, vid, pid);
	}

	if (dev_dir_opened) {
		(void)closedir(dev_dir);
	}

	return hid_dut_found ? EXIT_SUCCESS : EXIT_FAILURE;
}

int rebind_hid_driver(int vid, int pid)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	char i2c_dev_name[I2C_DEV_NAME_MAX] = {0};
	char driver_path[I2C_DEV_PATH_MAX] = {0};
	char driver_unbind[I2C_DEV_PATH_MAX] = {0};
	char driver_bind[I2C_DEV_PATH_MAX] = {0};

	if (EXIT_SUCCESS != auto_detect_hid_dut(vid, pid, i2c_dev_name, driver_path)) {
		return EXIT_FAILURE;
	}

	output(INFO, "Rebinding HID driver for %s to %s\n", i2c_dev_name, driver_path);

	snprintf(driver_unbind, I2C_DEV_PATH_MAX, "%s/unbind", driver_path);
	output(DEBUG, "Driver unbind path: %s\n", driver_unbind);

	FILE *fp = fopen(driver_unbind, "w");
	if (fp == NULL) {
		output(ERROR, "%s: Failed to open %s for writing. %s [%d]\n",
			   __func__, driver_unbind, strerror(errno), errno);
		return EXIT_FAILURE;
	}
	if (fprintf(fp, "%s", i2c_dev_name) < 0) {
		output(ERROR, "%s: Failed to write to %s. %s [%d]\n",
			   __func__, driver_unbind, strerror(errno), errno);
		fclose(fp);
		return EXIT_FAILURE;
	}
	fclose(fp);

	sleep_ms(DELAY_AFTER_DRIVER_UNBIND);

	snprintf(driver_bind, I2C_DEV_PATH_MAX, "%s/bind", driver_path);
	output(DEBUG, "Driver bind path: %s\n", driver_bind);
	fp = fopen(driver_bind, "w");
	if (fp == NULL) {
		output(ERROR, "%s: Failed to open %s for writing. %s [%d]\n",
			   __func__, driver_bind, strerror(errno), errno);
		return EXIT_FAILURE;
	}
	if (fprintf(fp, "%s", i2c_dev_name) < 0) {
		output(ERROR, "%s: Failed to write to %s. %s [%d]\n",
			   __func__, driver_bind, strerror(errno), errno);
		fclose(fp);
		return EXIT_FAILURE;
	}
	fclose(fp);

	sleep_ms(DELAY_AFTER_DRIVER_BIND);

	return EXIT_SUCCESS;
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

	if (hid_desc == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	}

	if (hid_desc_read) {
		output(DEBUG, "HID descriptor already read.\n");
		goto COPY;
	}

	if (EXIT_FAILURE == _get_hid_descriptor_from_data_block(&_hid_desc)) {
		return EXIT_FAILURE;
	}

COPY:
	memcpy((void*) hid_desc, (void*) &_hid_desc, sizeof(HID_Descriptor));
	hid_desc_read = true;
	return EXIT_SUCCESS;
}


static int _get_hid_descriptor_from_data_block(HID_Descriptor* hid_desc)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	uint8_t suspend_scan_cmd_data[] = {
			0x04, 0x06, 0x00, 0x08, 0x33, 0x2C, 0xC0};
	ReportData suspend_scan_cmd = {
			.data = suspend_scan_cmd_data,
			.len  = sizeof(suspend_scan_cmd_data)
	};

	uint8_t suspend_scan_rsp_data[HID_MAX_INPUT_REPORT_SIZE];
	ReportData suspend_scan_rsp = {
			.data = suspend_scan_rsp_data,
			.max_len = HID_MAX_INPUT_REPORT_SIZE
	};
	if (EXIT_SUCCESS != _pip_cmd_rsp(&suspend_scan_cmd, &suspend_scan_rsp)) {
		return EXIT_FAILURE;
	}

	uint8_t get_data_block_cmd_data[] =
		{ 0x04, 0x0B, 0x00, 0x08, 0x22, 0x00, 0x00, 0xFF, 0xFF, 0x03, 0xCD, 0xD3 };

	ReportData get_data_block_cmd = {
			.data = get_data_block_cmd_data,
			.len  = sizeof(get_data_block_cmd_data)
	};

	ReportData get_data_block_rsp = {
			.data = get_data_block_cmd_data,
			.max_len = HID_MAX_INPUT_REPORT_SIZE
	};
	if (EXIT_SUCCESS != _pip_cmd_rsp(&get_data_block_cmd, &get_data_block_rsp)) {
		return EXIT_FAILURE;
	}

	memcpy(hid_desc, &get_data_block_rsp.data[13], sizeof(HID_Descriptor));

	return EXIT_SUCCESS;
}

static int _pip_cmd_rsp(ReportData* report, ReportData* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (report == NULL || report->data == NULL || rsp == NULL || rsp->data == NULL)
	{
		output(ERROR, "%s: INVALID PARAMETERS\n", __func__);
		return EXIT_FAILURE;
	}

	int rc = EXIT_FAILURE;
	int fd = open(hidraw_sysfs_node_file, O_RDWR);
	if (fd < 0) {
		output(ERROR, "%s: Failed to open %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		return EXIT_FAILURE;
	}

	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_HID,
		"HID RAW CMD:", REPORT_TYPE_COMMAND, report);
	if (EXIT_SUCCESS != write_report(report, hidraw_sysfs_node_file)) {
		goto EXIT;
	}

	sleep_ms(AVG_DELAY_BETWEEN_CMD_AND_RSP);

	rsp->len = read(fd, rsp->data, rsp->max_len);
	if (rsp->len < 0) {
		output(ERROR, "%s: Failed to read from %s. %s [%d]\n", __func__,
				hidraw_sysfs_node_file, strerror(errno), errno);
		goto EXIT;
	} else if (rsp->len == 0) {
		output(ERROR, "%s: Zero bytes read from %s. Something went wrong.\n",
				__func__, hidraw_sysfs_node_file);
		goto EXIT;
	}

	bool m_rpt = rsp->data[1] & 0x01;
	while(m_rpt) {
		uint8_t tmp_buf[256];
		int read_len = read(fd, tmp_buf, sizeof(tmp_buf));
		if (read_len < 0) {
			output(ERROR, "%s: Failed to read from %s. %s [%d]\n", __func__,
					hidraw_sysfs_node_file, strerror(errno), errno);
			goto EXIT;
		} else if (read_len == 0) {
			output(ERROR, "%s: Zero bytes read from %s. Something went wrong.\n",
					__func__, hidraw_sysfs_node_file);
			goto EXIT;
		}

		m_rpt = tmp_buf[1] & 0x01;
		int data_len = read_len - 2;
		memcpy(&rsp->data[rsp->len], &tmp_buf[2], data_len);
		rsp->len += data_len;
	}

	rc = EXIT_SUCCESS;

EXIT:
	close(fd);
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
	self_pipe_create = true;

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
	report_buffer_create = true;

	if (0 != pthread_create(&report_reader_tid, NULL, _report_reader_thread,
			(void*) &report_id)) {
		output(ERROR,
				"%s: Failed to start thread for reading reports from %s. "
				"%s [%d]\n",
				__func__, hidraw_sysfs_node_file, strerror(errno), errno);
		rc = EXIT_FAILURE;
		goto RETURN;
	}
	report_reader_create = true;

	while (report_reader_thread_status
			== REPORT_READER_THREAD_STATUS_NOT_STARTED)
		sleep_ms(1);

	rc = EXIT_SUCCESS;

RETURN:
	if (rc != EXIT_SUCCESS) {
		stop_hidraw_report_reader();
	}

	return rc;
}

int stop_hidraw_report_reader()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_SUCCESS;

	if (!hidraw0_open || !self_pipe_create)
		return EXIT_SUCCESS;

	stop_reading = true;

	char stop_signal[] = "S";
	write(self_pipe_fd[SELF_PIPE_WRITE], stop_signal, 2);

	struct timeval start_time;
	gettimeofday(&start_time, 0);
	while (!time_limit_reached(&start_time, 5) &&
	    report_reader_thread_status != REPORT_READER_THREAD_STATUS_EXIT);

	if (report_reader_create) {
		report_reader_create = false;

		rc = pthread_join(report_reader_tid, NULL);
		if (rc == 0) {
			output(DEBUG,
				"The thread reading HIDRAW reports is terminated.\n");
		} else if (rc == ESRCH) {
			rc = EXIT_SUCCESS;
			output(DEBUG,
				"The thread reading HIDRAW reports has already terminated.\n");
		} else {
			output(ERROR,
				"%s: Failed to stop thread reading HIDRAW reports. %s [%d]\n",
					__func__, strerror(rc), rc);
		}
	}

	if (report_buffer_create) {
		report_buffer_create = false;

		for (int i = 0; i < REPORT_BUFFER_SIZE; i++) {
			free(report_buffer[i].report.data);
			report_buffer[i].report.data = NULL;
			report_buffer[i].ready = false;
		}
	}

	close(hidraw0_fd);
	hidraw0_open = false;
	close(self_pipe_fd[SELF_PIPE_READ]);
	close(self_pipe_fd[SELF_PIPE_WRITE]);
	self_pipe_create = false;

	return rc;
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
