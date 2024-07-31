/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "pip3.h"

#define AVG_DELAY_BETWEEN_CMD_AND_RSP 5 

#define MAX_TIMEOUT_BETWEEN_CMD_AND_RSP 7 

#define MAX_SEQ_NUM 0x07

#define TAG_BIT 1

char* PIP3_EXEC_NAMES[] = {
		[PIP3_EXEC_ROM] = "ROM Bootloader EXEC",
		[PIP3_EXEC_RAM] = "RAM Application EXEC"
};

char* PIP3_APP_SYS_MODE_NAMES[] = {
		[PIP3_APP_SYS_MODE_BOOT]            = "Bootup mode",
		[PIP3_APP_SYS_MODE_SCANNING]        = "Scanning mode",
		[PIP3_APP_SYS_MODE_DEEP_SLEEP]      = "Deep sleep mode",
		[PIP3_APP_SYS_MODE_TEST_CONFIG]     = "Test and configuration mode",
		[PIP3_APP_SYS_MODE_DEEP_STANDBY]    = "Deep standby"
};

char* PIP3_FW_CATEGORY_NAMES[] = {
		[PIP3_FW_CATEGORY_ID_TOUCH_FW]      = "Touch Firmware Category",
		[PIP3_FW_CATEGORY_ID_PROGRAMMER_FW] = "Programmer Firmware Category",
		[PIP3_FW_CATEGORY_ID_UTILITY_FW]    = "Utility Firmware Category",
};

char* PIP3_IMAGE_NAMES[] = {
		[PIP3_IMAGE_ID_PRIMARY]   = "Primary",
		[PIP3_IMAGE_ID_SECONDARY] = "Secondary",
		[PIP3_IMAGE_ID_ROM_BL]    = "ROM BL"
};

char* PIP3_IOCTL_CODE_LABELS[] = {
		[PIP3_IOCTL_CODE_ERASE_FILE]         = "Erase file",
		[PIP3_IOCTL_CODE_SEEK_FILE_POINTERS] = "Seek file pointers",
		[PIP3_IOCTL_CODE_AES_CONTROL]        = "AES control",
		[PIP3_IOCTL_CODE_FILE_STATS]         = "File stats",
		[PIP3_IOCTL_CODE_FILE_CRC]           = "File CRC",
};

char* PIP3_PROCESSOR_NAMES[] = {
		[PIP3_PROCESSOR_ID_PRIMARY] = "Primary",
		[PIP3_PROCESSOR_ID_AUX_MCU] = "AUX MCU",
};

static Channel* active_channel = NULL;
static uint16_t hid_max_output_report_len;
static uint16_t hid_max_input_report_len;

static bool        async_debug_data_mode_activated = false;
static PIP3_Cmd_ID async_debug_data_mode_cmd_id;
static uint8_t     async_debug_data_mode_seq;

static int _verify_pip3_rsp_report(HID_Report_ID report_id, uint8_t seq,
		PIP3_Cmd_ID cmd_id, const HID_Input_PIP3_Response* rsp);

int (*send_report_via_channel)(const ReportData* report);

Poll_Status (*get_report_via_channel)(ReportData* report, bool apply_timeout,
		long double timeout_val);

int do_pip3_command(ReportData* cmd, ReportData* rsp)
{
	const HID_Output_PIP3_Command* output_report;
	bool more_reports = false;
	size_t payload_len = 0;
	size_t remaining_payload_len = 0;
	int rc;
	Poll_Status read_rc;
	ReportData rsp_report;

	output_report = (HID_Output_PIP3_Command*) cmd->data;

	rsp_report.data = NULL;
	rsp_report.max_len = hid_max_input_report_len - 2;
	rsp_report.data = (uint8_t*) malloc(rsp_report.max_len * sizeof(uint8_t));
	if (rsp_report.data == NULL) {
		output(ERROR, "%s: Memory allocation failed. %s [%d].\n", __func__,
				strerror(errno), errno);
		return EXIT_FAILURE;
	}

	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_HID,
			PIP3_CMD_NAMES[output_report->cmd_id], REPORT_TYPE_COMMAND, cmd);
	rc = send_report_via_channel(cmd);
	if (rc != EXIT_SUCCESS) {
		goto RETURN;
	}

	sleep_ms(AVG_DELAY_BETWEEN_CMD_AND_RSP);

	do {
		const HID_Input_PIP3_Response* input_report;

		memset(rsp_report.data, 0, sizeof(rsp_report.max_len));
		read_rc = get_report_via_channel(&rsp_report, true,
				MAX_TIMEOUT_BETWEEN_CMD_AND_RSP);
		switch (read_rc) {
		case POLL_STATUS_GOT_DATA:
			break;
		case POLL_STATUS_TIMEOUT:
			output(ERROR,
					"%s: Timed-Out waiting for PIP3 %s Response.\n",
					__func__, PIP3_CMD_NAMES[output_report->cmd_id]);
			rc = EXIT_FAILURE;
			break;
		case POLL_STATUS_ERROR:
			output(ERROR,
					"%s: Unexpected error occurred while attempting to retrieve"
					" the PIP3 %s Response.\n",
					__func__, PIP3_CMD_NAMES[output_report->cmd_id]);
			rc = EXIT_FAILURE;
			break;
		default:
			output(ERROR,
					"%s: Unexpected 'Poll_Status' enum value (%d) for pending "
					"PIP3 %s Response.\n",
					__func__, read_rc, PIP3_CMD_NAMES[output_report->cmd_id]);
			rc = EXIT_FAILURE;
		}

		if (rc == EXIT_SUCCESS) {
			input_report = (HID_Input_PIP3_Response*) rsp_report.data;
			rc = _verify_pip3_rsp_report(HID_REPORT_ID_SOLICITED_RESPONSE,
					output_report->seq, output_report->cmd_id, input_report);
		}

		if (rc != EXIT_SUCCESS) {
			goto RETURN;
		}

		more_reports = input_report->more_reports;
		if (input_report->first_report == 1) {
			payload_len = ((input_report->payload_len_msb << 8)
					| input_report->payload_len_lsb);
			remaining_payload_len = payload_len;
			output(DEBUG, "Payload Length: %u\n", payload_len);
			output_debug_report(REPORT_DIRECTION_INCOMING_FROM_DUT,
					REPORT_FORMAT_HID, PIP3_CMD_NAMES[output_report->cmd_id],
					REPORT_TYPE_RESPONSE, &rsp_report);
		} else {
			output_debug_report(REPORT_DIRECTION_INCOMING_FROM_DUT,
					REPORT_FORMAT_HID, "(continued response)",
					REPORT_TYPE_RESPONSE, &rsp_report);
		}

		if (rsp != NULL) {
			size_t rsp_report_len = rsp_report.len - 2;

			size_t copy_len = ((remaining_payload_len > rsp_report_len)
					? rsp_report_len : remaining_payload_len);

			if (payload_len > rsp->max_len) {
				output(ERROR, "%s: The response payload is larger (%u bytes) "
						"than the maximum size supported (%u bytes).\n",
						__func__, payload_len, rsp->max_len);
				rc = EXIT_FAILURE;
			} else if (copy_len + rsp->len > payload_len) {
				output(ERROR, "%s: The response reports added up to a larger "
						"total paylaod (%u) than expected (%u bytes).\n",
						__func__, copy_len + rsp->len, payload_len);
				rc = EXIT_FAILURE;
			} else {
				memcpy(&(rsp->data[rsp->len]),
						&(rsp_report.data[
								HID_INPUT_PIP3_RSP_PAYLOAD_START_BYTE_INDEX]),
						copy_len);

				rsp->len += copy_len;
				remaining_payload_len -= copy_len;
			}
		}
	} while (rc == EXIT_SUCCESS && more_reports);

RETURN:
	free(rsp_report.data);
	return rc;
}

int do_pip3_calibrate_cmd(uint8_t seq_num, uint8_t calibration_mode,
		uint8_t data_0, uint8_t data_1, uint8_t data_2)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_Calibrate) - 1;
	PIP3_Cmd_Payload_Calibrate cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_CALIBRATE,
					.resp               = 0
			},
			.calibration_mode = calibration_mode,
			.data_0           = data_0,
			.data_1           = data_1,
			.data_2           = data_2,
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	PIP3_Rsp_Payload_Calibrate rsp_data;
	ReportData rsp = {
			.data        = (uint8_t*) &rsp_data,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_Calibrate)
	};

	return do_pip3_command(&cmd, &rsp);
}

int do_pip3_initialize_baselines_cmd(uint8_t seq_num, uint8_t data_id_mask,
		PIP3_Rsp_Payload_InitializeBaselines* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_InitializeBaselines) - 1;
	PIP3_Cmd_Payload_InitializeBaselines cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             =
									(uint8_t) PIP3_CMD_ID_INITIALIZE_BASELINE,
					.resp               = 0
			},
			.data_id_mask = data_id_mask,
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_InitializeBaselines)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_file_close_cmd(uint8_t seq_num, uint8_t file_handle,
		PIP3_Rsp_Payload_FileClose* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_FileClose) - 1;
	PIP3_Cmd_Payload_FileClose cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_FILE_CLOSE,
					.resp               = 0
			},
			.file_handle = file_handle,
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_FileClose)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_file_ioctl_erase_file_cmd(uint8_t seq_num, uint8_t file_handle,
		PIP3_Rsp_Payload_FileIOCTL_EraseFile* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_FileIOCTL_EraseFile) - 1;
	PIP3_Cmd_Payload_FileIOCTL_EraseFile cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_FILE_IOCTL,
					.resp               = 0
			},
			.file_handle = file_handle,
			.ioctl_code  = (uint8_t) PIP3_IOCTL_CODE_ERASE_FILE
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_FileIOCTL_EraseFile)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_file_open_cmd(uint8_t seq_num, uint8_t file_num,
		PIP3_Rsp_Payload_FileOpen* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_FileOpen) - 1;
	PIP3_Cmd_Payload_FileOpen cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_FILE_OPEN,
					.resp               = 0
			},
			.file_num = file_num,
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_FileOpen)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_file_read_cmd(uint8_t seq_num, uint8_t file_handle,
		uint16_t read_len, PIP3_Rsp_Payload_FileRead* rsp, size_t max_rsp_size)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_FileRead) - 1;
	PIP3_Cmd_Payload_FileRead cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_FILE_READ,
					.resp               = 0
			},
			.file_handle  = file_handle,
			.read_len_lsb = read_len & 0xFF,
			.read_len_msb = read_len >> 8
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = NULL,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = max_rsp_size
	};
	uint16_t rsp_crc;
	uint8_t rsp_crc_msb;
	uint8_t rsp_crc_lsb;

	_rsp.data = calloc(_rsp.max_len, 1);
	if (_rsp.data == NULL) {
		output(ERROR, "%s: Memory allocation failed.\n", __func__);
		return EXIT_FAILURE;
	}

	rc = do_pip3_command(&cmd, &_rsp);
	if (_rsp.len < PIP3_RSP_MIN_LEN) {
		output(ERROR, "%s: PIP3 FILE_READ response is shorter than the min "
				"possible PIP3 reponse (% bytes).\n",
				__func__, PIP3_RSP_MIN_LEN);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	memcpy((void*) &rsp->header, (void*) _rsp.data, sizeof(PIP3_Rsp_Header));

	memcpy((void*) rsp->data, (void*) &_rsp.data[sizeof(PIP3_Rsp_Header)],
			read_len);

	rsp->footer.crc_msb = _rsp.data[_rsp.len - 2];
	rsp->footer.crc_lsb = _rsp.data[_rsp.len - 1];

	rsp_crc = calculate_crc16_ccitt(0xFFFF, _rsp.data, _rsp.len - 2);
	rsp_crc_msb = rsp_crc >> 8;
	rsp_crc_lsb = rsp_crc & 0xFF;

	if (rsp->footer.crc_msb != rsp_crc_msb
			|| rsp->footer.crc_lsb != rsp_crc_lsb) {
		output(ERROR,
				"Unexpected PIP3 Response CRC:\n"
				"\tReceived   = %02X %02X\n"
				"\tCalculated = %02X %02X\n",
				rsp->footer.crc_msb, rsp->footer.crc_lsb,
				rsp_crc_msb, rsp_crc_lsb);
		rc = EXIT_FAILURE;
	}

RETURN:
	free(_rsp.data);
	return rc;
}

int do_pip3_file_write_cmd(uint8_t seq_num, uint8_t file_handle, ByteData* data)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (data == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	ReportData cmd = { .data = NULL };
	size_t data_part_start_index = 0;
	bool error_occurred = false;
	size_t max_data_per_cmd_len;
	int rc = EXIT_FAILURE;
	size_t remaining_data_len = data->len;
	size_t remaining_num_of_writes;

	cmd.max_len = hid_max_output_report_len - 2;
	max_data_per_cmd_len = cmd.max_len - PIP3_FILE_WRITE_CMD_WITHOUT_DATA_LEN;
	cmd.data = malloc(cmd.max_len);
	if (cmd.data == NULL) {
		output(ERROR, "%s: Memory allocation failed.\n", __func__);
		return EXIT_FAILURE;
	}

	remaining_num_of_writes =
			(data->len + (max_data_per_cmd_len - 1)) / max_data_per_cmd_len;

	while (remaining_data_len > 0 && !error_occurred) {
		uint16_t cmd_crc;
		size_t data_part_len;
		PIP3_Rsp_Payload_FileWrite rsp;
		ReportData _rsp = {
				.data        = (uint8_t*) &rsp,
				.len         = 0,
				.index       = 0,
				.num_records = 0,
				.max_len     = sizeof(PIP3_Rsp_Payload_FileWrite)
		};

		data_part_len = ((remaining_data_len > max_data_per_cmd_len)
				? max_data_per_cmd_len : remaining_data_len);

		cmd.len = data_part_len + PIP3_FILE_WRITE_CMD_WITHOUT_DATA_LEN;
		if (cmd.len > cmd.max_len) {
			output(ERROR,
					"%s: PIP3 Command length (%u bytes) is too large per the "
					"HID descriptor (%u bytes).\n",
					__func__, cmd.len, hid_max_output_report_len);
			rc = EXIT_FAILURE;
			error_occurred = true;
		}

		if (!error_occurred) {
			remaining_data_len -= data_part_len;

			PIP3_Cmd_Payload_FileWrite* cmd_data =
					(PIP3_Cmd_Payload_FileWrite*) cmd.data;
			uint16_t cmd_payload_len = (uint16_t) cmd.len - 1;

			cmd_data->header.report_id = HID_REPORT_ID_COMMAND;
			cmd_data->header.payload_len_lsb = cmd_payload_len & 0xFF;
			cmd_data->header.payload_len_msb = cmd_payload_len >> 8;
			cmd_data->header.seq = seq_num;
			cmd_data->header.tag = TAG_BIT;
			cmd_data->header.more_data = 0;
			cmd_data->header.reserved_section_1 = 0;
			cmd_data->header.cmd_id = (uint8_t) PIP3_CMD_ID_FILE_WRITE;
			cmd_data->header.resp = 0;
			cmd_data->file_handle = file_handle;

			memcpy((void*) &cmd.data[6],
					(void*) &data->data[data_part_start_index], data_part_len);
			cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
					cmd.len - 3);
			cmd.data[cmd.len - 2] = cmd_crc >> 8;
			cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

			rc = do_pip3_command(&cmd, &_rsp);
			if (EXIT_SUCCESS != rc) {
				output(ERROR,
						"%s: Aborting the remaining %u FILE_WRITE commands that"
						" are pending execution.\n",
						__func__, remaining_num_of_writes - 1);
				error_occurred = true;
			} else {
				data_part_start_index += data_part_len;
				remaining_num_of_writes--;
				output(DEBUG,
					"Remaining number of FILE_WRITE commands to execute: %u.\n",
					remaining_num_of_writes);
			}
		}
	}

	free(cmd.data);
	return rc;
}

int do_pip3_get_self_test_results_cmd(uint8_t seq_num, uint8_t self_test_id,
		PIP3_Rsp_Payload_GetSelfTestResults* rsp, size_t max_rsp_size)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_GetSelfTestResults) - 1;
	PIP3_Cmd_Payload_GetSelfTestResults cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             =
									(uint8_t) PIP3_CMD_ID_GET_SELF_TEST_RESULTS,
					.resp               = 0
			},
			.read_offset_lsb = 0x00,
			.read_offset_msb = 0x00,
			.read_len_lsb    = 0xFF,
			.read_len_msb    = 0xFF,
			.self_test_id    = self_test_id
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	size_t arl;
	int rc = EXIT_FAILURE;
	ReportData _rsp = {
			.data        = NULL,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = max_rsp_size
	};
	uint16_t rsp_crc;
	uint8_t rsp_crc_msb;
	uint8_t rsp_crc_lsb;

	_rsp.data = (uint8_t*) calloc(max_rsp_size, 1);
	if (_rsp.data == NULL) {
		output(ERROR, "%s: Memory allocation failed.\n", __func__);
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS != do_pip3_command(&cmd, &_rsp)) {
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	memcpy((void*) &rsp->header, (void*) _rsp.data, sizeof(PIP3_Rsp_Header));

	rsp->self_test_id =
			((PIP3_Rsp_Payload_GetSelfTestResults*) _rsp.data)->self_test_id;
	if (rsp->self_test_id != self_test_id) {
		output(ERROR,
				"Self-Test ID mismatch between PIP3 Command and Response:\n"
				"\tReceived = 0x%02X\n"
				"\tExpected = 0x%02X\n",
				rsp->self_test_id, self_test_id);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	rsp->data_format_id =
			((PIP3_Rsp_Payload_GetSelfTestResults*) _rsp.data)->data_format_id;
	rsp->arl_lsb =
			((PIP3_Rsp_Payload_GetSelfTestResults*) _rsp.data)->arl_lsb;
	rsp->arl_msb =
			((PIP3_Rsp_Payload_GetSelfTestResults*) _rsp.data)->arl_msb;

	arl = (rsp->arl_msb << 8) | rsp->arl_lsb;

	memcpy((void*) rsp->data, (void*) &_rsp.data[sizeof(PIP3_Rsp_Header) + 4],
			arl);

	rsp->footer.crc_msb = _rsp.data[_rsp.len - 2];
	rsp->footer.crc_lsb = _rsp.data[_rsp.len - 1];

	rsp_crc = calculate_crc16_ccitt(0xFFFF, _rsp.data, _rsp.len - 2);
	rsp_crc_msb = rsp_crc >> 8;
	rsp_crc_lsb = rsp_crc & 0xFF;

	if (rsp->footer.crc_msb != rsp_crc_msb
			|| rsp->footer.crc_lsb != rsp_crc_lsb) {
		output(ERROR,
				"Unexpected PIP3 Response CRC:\n"
				"\tReceived   = %02X %02X\n"
				"\tCalculated = %02X %02X\n",
				rsp->footer.crc_msb, rsp->footer.crc_lsb,
				rsp_crc_msb, rsp_crc_lsb);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	rc = EXIT_SUCCESS;

RETURN:
	free(_rsp.data);
	return rc;
}

int do_pip3_get_sysinfo_cmd(uint8_t seq_num, PIP3_Rsp_Payload_GetSysinfo* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_GetSysinfo) - 1;
	PIP3_Cmd_Payload_GetSysinfo cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_GET_SYSINFO,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(
			0xFFFF, &(cmd.data[1]), cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_GetSysinfo)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_load_self_test_param_cmd(uint8_t seq_num, uint8_t self_test_id,
		ByteData* param_data)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (param_data == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	ReportData cmd = { .data = NULL };
	uint16_t load_offset = 0;
	size_t param_data_part_start_index = 0;
	bool error_occurred = false;
	size_t max_param_data_per_cmd_len;
	int rc = EXIT_FAILURE;
	size_t remaining_param_data_len = param_data->len;
	size_t remaining_num_of_cmds;

	cmd.max_len = hid_max_output_report_len - 2;
	max_param_data_per_cmd_len = cmd.max_len
			- PIP3_LOAD_SELF_TEST_PARAM_CMD_WITHOUT_PARAM_DATA_LEN;
	cmd.data = malloc(cmd.max_len);
	if (cmd.data == NULL) {
		output(ERROR, "%s: Memory allocation failed.\n", __func__);
		return EXIT_FAILURE;
	}

	remaining_num_of_cmds =
			(param_data->len + (max_param_data_per_cmd_len - 1))
			/ max_param_data_per_cmd_len;

	while (remaining_param_data_len > 0 && !error_occurred) {
		uint16_t cmd_crc;
		size_t param_data_part_len;
		PIP3_Rsp_Payload_LoadSelfTestParam rsp;
		ReportData _rsp = {
				.data        = (uint8_t*) &rsp,
				.len         = 0,
				.index       = 0,
				.num_records = 0,
				.max_len     = sizeof(PIP3_Rsp_Payload_LoadSelfTestParam)
		};

		param_data_part_len = (
				(remaining_param_data_len > max_param_data_per_cmd_len)
				? max_param_data_per_cmd_len : remaining_param_data_len);

		cmd.len = param_data_part_len
				+ PIP3_LOAD_SELF_TEST_PARAM_CMD_WITHOUT_PARAM_DATA_LEN;
		if (cmd.len > cmd.max_len) {
			output(ERROR,
					"%s: PIP3 Command length (%u bytes) is too large per the "
					"HID descriptor (%u bytes).\n",
					__func__, cmd.len, hid_max_output_report_len);
			rc = EXIT_FAILURE;
			error_occurred = true;
		}

		if (!error_occurred) {
			remaining_param_data_len -= param_data_part_len;

			PIP3_Cmd_Payload_LoadSelfTestParam* cmd_data =
					(PIP3_Cmd_Payload_LoadSelfTestParam*) cmd.data;
			uint16_t cmd_payload_len = (uint16_t) cmd.len - 1;

			cmd_data->header.report_id = HID_REPORT_ID_COMMAND;
			cmd_data->header.payload_len_lsb = cmd_payload_len & 0xFF;
			cmd_data->header.payload_len_msb = cmd_payload_len >> 8;
			cmd_data->header.seq = seq_num;
			cmd_data->header.tag = TAG_BIT;
			cmd_data->header.more_data = 0;
			cmd_data->header.reserved_section_1 = 0;
			cmd_data->header.cmd_id =
					(uint8_t) PIP3_CMD_ID_LOAD_SELF_TEST_PARAM;
			cmd_data->load_offset_lsb = load_offset & 0xFF;
			cmd_data->load_offset_msb = load_offset >> 8;
			cmd_data->load_len_lsb = param_data_part_len & 0xFF;
			cmd_data->load_len_msb = (uint8_t) param_data_part_len >> 8;
			cmd_data->self_test_id = self_test_id;
			memcpy((void*) &cmd.data[
					PIP3_LOAD_SELF_TEST_PARAM_CMD_PARAM_DATA_START_BYTE_INDEX],
					(void*) &param_data->data[param_data_part_start_index],
					param_data_part_len);
			cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
					cmd.len - 3);
			cmd.data[cmd.len - 2] = cmd_crc >> 8;
			cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

			rc = do_pip3_command(&cmd, &_rsp);

			load_offset += param_data_part_len;
			param_data_part_start_index += param_data_part_len;
			remaining_num_of_cmds--;
			output(DEBUG,
					"Remaining number of LOAD_SELF_TEST_PARAM commands to "
					"execute: %u.\n",
					remaining_num_of_cmds);
		}
	}

	free(cmd.data);
	return rc;
}

int do_pip3_resume_scanning_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_ResumeScanning* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_ResumeScanning) - 1;
	PIP3_Cmd_Payload_ResumeScanning cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_RESUME_SCAN,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_ResumeScanning)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_run_self_test_cmd(uint8_t seq_num, uint8_t self_test_id,
		PIP3_Rsp_Payload_RunSelfTest* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_RunSelfTest) - 1;
	PIP3_Cmd_Payload_RunSelfTest cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_RUN_SELF_TEST,
					.resp               = 0
			},
			.self_test_id = self_test_id
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_RunSelfTest)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_start_tracking_heatmap_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_StartTrackingHeatmap* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	} else if (async_debug_data_mode_activated) {
		output(ERROR,
				"%s: Asynchronous debug data mode was already activated via the"
				" PIP3 %s command.\n",
				__func__, PIP3_CMD_NAMES[async_debug_data_mode_cmd_id]);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len =
			sizeof(PIP3_Cmd_Payload_StartTrackingHeatmap) - 1;
	PIP3_Cmd_Payload_StartTrackingHeatmap cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             =
								(uint8_t) PIP3_CMD_ID_START_TRACKING_HEATMAP,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_StartTrackingHeatmap)
	};

	if (EXIT_SUCCESS != do_pip3_command(&cmd, &_rsp)) {
		return EXIT_FAILURE;
	}

	async_debug_data_mode_activated = true;
	async_debug_data_mode_cmd_id = cmd_data.header.cmd_id;
	async_debug_data_mode_seq = cmd_data.header.seq;

	return EXIT_SUCCESS;
}

int do_pip3_status_cmd(uint8_t seq_num, PIP3_Rsp_Payload_Status* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_Status) - 1;
	PIP3_Cmd_Payload_Status cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_STATUS,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_Status)
	};

	return do_pip3_command(&cmd, &_rsp);
}

int do_pip3_stop_async_debug_data_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_StopAsyncDebugData* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len =
			sizeof(PIP3_Cmd_Payload_StopAsyncDebugData) - 1;
	PIP3_Cmd_Payload_StopAsyncDebugData cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             =
									(uint8_t) PIP3_CMD_ID_STOP_ASYNC_DEBUG_DATA,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_StopAsyncDebugData)
	};

	if (EXIT_SUCCESS != do_pip3_command(&cmd, &_rsp)) {
		return EXIT_FAILURE;
	}

	async_debug_data_mode_activated = false;

	return EXIT_SUCCESS;
}

int do_pip3_suspend_scanning_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_SuspendScanning* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_SuspendScanning) - 1;
	PIP3_Cmd_Payload_SuspendScanning cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_SUSPEND_SCAN,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(
			0xFFFF, &(cmd.data[1]), cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_SuspendScanning)
	};

	return do_pip3_command(&cmd, &_rsp);
}

#define DELAY_FOR_SWITCH_ACTIVE_PROCESS_CMD_MSECS 2

int do_pip3_switch_active_processor_cmd(uint8_t seq_num,
		PIP3_Processor_ID processor_id, uint8_t switch_data)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	output(DEBUG, "Switching to the %s processor.\n",
			PIP3_PROCESSOR_NAMES[processor_id]);
	int rc = EXIT_FAILURE;

	uint16_t cmd_payload_len =
			sizeof(PIP3_Cmd_Payload_SwitchActiveProcessor) - 1;
	PIP3_Cmd_Payload_SwitchActiveProcessor cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             =
							(uint8_t) PIP3_CMD_ID_SWITCH_ACTIVE_PROCESSOR,
					.resp               = 0
			},
			.processor_id = (uint8_t) processor_id,
			.switch_data  = switch_data
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(
			0xFFFF, &(cmd.data[1]), cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_HID,
			PIP3_CMD_NAMES[PIP3_CMD_ID_SWITCH_ACTIVE_PROCESSOR],
			REPORT_TYPE_COMMAND, &cmd);
	rc = send_report_via_channel(&cmd);

	output(DEBUG,
			"Waiting %u milliseconds to give the DUT enough time to switch "
			"processes.\n",
			DELAY_FOR_SWITCH_ACTIVE_PROCESS_CMD_MSECS);
	sleep_ms(DELAY_FOR_SWITCH_ACTIVE_PROCESS_CMD_MSECS);

	output(DEBUG, "%s: Leaving.\n", __func__);
	return rc;
}

#define DELAY_FOR_SWITCH_IMAGE_CMD_SECS 2

int do_pip3_switch_image_cmd(uint8_t seq_num, PIP3_Image_ID image_id)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	output(DEBUG, "Switching to the %s image.\n", PIP3_IMAGE_NAMES[image_id]);
	int rc = EXIT_FAILURE;

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_SwitchImage) - 1;
	PIP3_Cmd_Payload_SwitchImage cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_SWITCH_IMAGE,
					.resp               = 0
			},
			.image_id = (uint8_t) image_id
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(
			0xFFFF, &(cmd.data[1]), cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_HID,
			PIP3_CMD_NAMES[PIP3_CMD_ID_SWITCH_IMAGE], REPORT_TYPE_COMMAND,
			&cmd);
	rc = send_report_via_channel(&cmd);

	output(DEBUG,
			"Waiting %u seconds to give the DUT enough time to switch "
			"images.\n",
			DELAY_FOR_SWITCH_IMAGE_CMD_SECS);
	sleep(DELAY_FOR_SWITCH_IMAGE_CMD_SECS);

	output(DEBUG, "%s: Leaving.\n", __func__);
	return rc;
}

int do_pip3_version_cmd(uint8_t seq_num, PIP3_Rsp_Payload_Version* rsp)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (rsp == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (seq_num > MAX_SEQ_NUM) {
		output(ERROR,
				"%s: The sequence number must be less <= 7 (%u was given).\n",
				__func__, seq_num);
		return EXIT_FAILURE;
	}

	uint16_t cmd_payload_len = sizeof(PIP3_Cmd_Payload_Version) - 1;
	PIP3_Cmd_Payload_Version cmd_data = {
			.header = {
					.report_id          = HID_REPORT_ID_COMMAND,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.more_data          = 0,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP3_CMD_ID_VERSION,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[1]),
			cmd.len - 3);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP3_Rsp_Payload_Version)
	};

	return do_pip3_command(&cmd, &_rsp);
}

bool is_pip3_api_active()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	return active_channel != NULL && active_channel->type != CHANNEL_TYPE_NONE;
}

Poll_Status get_pip3_unsolicited_async_rsp(ReportData* rsp, bool apply_timeout,
		long double timeout_val)
{
	bool more_reports = false;
	size_t payload_len = 0;
	size_t remaining_payload_len = 0;
	Poll_Status rc;
	ReportData rsp_report;

	if (!async_debug_data_mode_activated) {
		output(ERROR,
				"%s: No asynchronous debug data mode has been activated yet.\n",
				__func__);
		return POLL_STATUS_ERROR;
	}

	rsp_report.data = NULL;
	rsp_report.max_len = hid_max_input_report_len - 2;
	rsp_report.data = (uint8_t*) malloc(rsp_report.max_len * sizeof(uint8_t));
	if (rsp_report.data == NULL) {
		output(ERROR, "%s: Memory allocation failed. %s [%d].\n", __func__,
				strerror(errno), errno);
		return POLL_STATUS_ERROR;
	}

	do {
		const HID_Input_PIP3_Response* input_report;

		memset(rsp_report.data, 0, sizeof(rsp_report.max_len));
		rc = get_report_via_channel(&rsp_report, apply_timeout, timeout_val);

		if (rc == POLL_STATUS_GOT_DATA) {
			input_report = (HID_Input_PIP3_Response*) rsp_report.data;
			if (EXIT_SUCCESS != _verify_pip3_rsp_report(
					HID_REPORT_ID_UNSOLICITED_RESPONSE,
					async_debug_data_mode_seq, async_debug_data_mode_cmd_id,
					input_report)) {
				rc = POLL_STATUS_ERROR;
			}
		}

		if (rc != POLL_STATUS_GOT_DATA) {
			goto RETURN;
		}

		more_reports = input_report->more_reports;
		if (input_report->first_report == 1) {
			payload_len = ((input_report->payload_len_msb << 8)
					| input_report->payload_len_lsb);
			remaining_payload_len = payload_len;
			output(DEBUG, "Payload Length: %u\n", payload_len);
			output_debug_report(REPORT_DIRECTION_INCOMING_FROM_DUT,
					REPORT_FORMAT_HID,
					PIP3_CMD_NAMES[async_debug_data_mode_cmd_id],
					REPORT_TYPE_UNSOLICTED_RESPONSE, &rsp_report);
		} else {
			output_debug_report(REPORT_DIRECTION_INCOMING_FROM_DUT,
					REPORT_FORMAT_HID, "(continued response)",
					REPORT_TYPE_UNSOLICTED_RESPONSE, &rsp_report);
		}

		if (rsp != NULL) {
			size_t rsp_report_len = rsp_report.len - 2;

			size_t copy_len = ((remaining_payload_len > rsp_report_len)
					? rsp_report_len : remaining_payload_len);

			if (payload_len > rsp->max_len) {
				output(ERROR, "%s: The reponse payload is larger (%u bytes) "
						"than the maximum size supported (%u bytes).\n",
						__func__, payload_len, rsp->max_len);
				rc = POLL_STATUS_ERROR;
			} else if (copy_len + rsp->len > payload_len) {
				output(ERROR, "%s: The response reports added up to a larger "
						"total paylaod (%u) than expected (%u bytes).\n",
						__func__, copy_len + rsp->len, payload_len);
				rc = POLL_STATUS_ERROR;
			} else {
				memcpy(&(rsp->data[rsp->len]),
						&(rsp_report.data[
								HID_INPUT_PIP3_RSP_PAYLOAD_START_BYTE_INDEX]),
						copy_len);

				rsp->len += copy_len;
				remaining_payload_len -= copy_len;
			}
		}
	} while (rc == POLL_STATUS_GOT_DATA && more_reports);

RETURN:
	free(rsp_report.data);
	return rc;
}

int setup_pip3_api(Channel* channel, HID_Report_ID report_id)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	HID_Descriptor hid_desc;

	if (channel == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (active_channel != NULL
			&& active_channel->type != channel->type) {
		output(ERROR,
				"%s: The API is already configured to use the %s channel.\n",
				__func__, CHANNEL_TYPE_NAMES[channel->type]);
		return EXIT_FAILURE;
	} else if (active_channel != NULL
			&& active_channel->type == channel->type) {
		output(DEBUG, "The API is already configured to use the %s channel.\n",
				CHANNEL_TYPE_NAMES[channel->type]);
		return EXIT_SUCCESS;
	}

	switch (channel->type) {
	case CHANNEL_TYPE_HIDRAW:
	case CHANNEL_TYPE_TTDL:
		break;

	default:
		output(ERROR, "%s: Given an invalid/unsupported channel type.\n",
				__func__);
		return EXIT_FAILURE;
	}

	active_channel = channel;

	output(DEBUG, "Using the %s channel type.\n",
			CHANNEL_TYPE_NAMES[active_channel->type]);

	if (EXIT_SUCCESS != active_channel->get_hid_descriptor(&hid_desc)) {
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS != active_channel->setup(report_id)) {
		return EXIT_FAILURE;
	}

	send_report_via_channel = active_channel->send_report;
	get_report_via_channel = active_channel->get_report;

	hid_max_input_report_len = hid_desc.max_input_len;
	hid_max_output_report_len = hid_desc.max_output_len;
	output(DEBUG, "HID Max Input Report length: %u bytes.\n",
			hid_max_input_report_len);
	output(DEBUG, "HID Max Output Report length: %u bytes.\n",
			hid_max_output_report_len);

	return EXIT_SUCCESS;
}

int teardown_pip3_api()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (active_channel == NULL) {
		output(DEBUG, "API is already inactive.\n");
		return EXIT_SUCCESS;
	}

	PIP3_Rsp_Payload_ResumeScanning resume_scanning_rsp;
	int rc = do_pip3_resume_scanning_cmd(0x00, &resume_scanning_rsp);

	active_channel->teardown();
	active_channel = NULL;

	return rc;
}

static int _verify_pip3_rsp_report(HID_Report_ID report_id, uint8_t seq,
		PIP3_Cmd_ID cmd_id, const HID_Input_PIP3_Response* rsp)
{
	if (rsp->report_id != report_id) {
		output(ERROR,
				"%s: Unexpected PIP3 Report ID: 0x%02X (expected 0x%02X)\n",
				__func__, rsp->report_id, report_id);
		return EXIT_FAILURE;
	}

	if (rsp->first_report == 1) {
		if (cmd_id != rsp->cmd_id) {
			output(ERROR,
					"%s: Got response for unexpected %s (0x%02X) command.\n",
					__func__, PIP3_CMD_NAMES[rsp->cmd_id],
					rsp->cmd_id);
			return EXIT_FAILURE;
		}

		if (report_id == HID_REPORT_ID_SOLICITED_RESPONSE) {
			if (seq != rsp->seq) {
				output(ERROR,
						"%s: The Command SEQ (%u) and Response SEQ (%u) do not "
						"match.\n",
						__func__, seq, rsp->seq);
				return EXIT_FAILURE;
			}

			if (rsp->resp != 1) {
				output(ERROR,
						"%s: RESP bit must always be set for PIP3 solicited"
						"asynchronous responses (Report ID = 0x%02X).\n",
						__func__, report_id);
				return EXIT_FAILURE;
			}

			const PIP3_Rsp_Header* pip3_rsp_header = (
					(const PIP3_Rsp_Header*) &(((const uint8_t*) rsp)[
								HID_INPUT_PIP3_RSP_PAYLOAD_START_BYTE_INDEX]));

			if (pip3_rsp_header->status_code != PIP3_STATUS_CODE_SUCCESS) {
				output(ERROR, "%s: PIP3 %s (0x%02X) command failed: %s.\n",
						__func__, PIP3_CMD_NAMES[cmd_id], cmd_id,
						PIP3_STATUS_CODE_LABELS[pip3_rsp_header->status_code]);
				return EXIT_FAILURE;
			}
		} else {
			if (rsp->resp != 0) {
				output(ERROR,
						"%s: RESP bit must always be unset for PIP3 unsolicited"
						"asynchronous responses (Report ID = 0x%02X).\n",
						__func__, report_id);
				return EXIT_FAILURE;
			}
		}
	}

	return EXIT_SUCCESS;
}
