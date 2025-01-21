/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "pip2.h"

#define AVG_DELAY_BETWEEN_CMD_AND_RSP 5 
#define FILE_IOCTL_ERASE_DELAY_BETWEEN_CMD_AND_RSP 3 

#define MAX_TIMEOUT_BETWEEN_CMD_AND_RSP 3 
#define DELAY_FOR_RESET  2 

#define MAX_SEQ_NUM 0x07

#define TAG_BIT 1

char* PIP2_EXEC_NAMES[] = {
		[PIP2_EXEC_ROM] = "ROM Bootloader EXEC",
		[PIP2_EXEC_RAM] = "RAM Application EXEC"
};

char* PIP2_APP_SYS_MODE_NAMES[] = {
		[PIP2_APP_SYS_MODE_BOOT]            = "Bootup mode",
		[PIP2_APP_SYS_MODE_SCANNING]        = "Scanning mode",
		[PIP2_APP_SYS_MODE_DEEP_SLEEP]      = "Deep sleep mode",
		[PIP2_APP_SYS_MODE_TEST_CONFIG]     = "Test and configuration mode",
		[PIP2_APP_SYS_MODE_DEEP_STANDBY]    = "Deep standby"
};

char* PIP2_IOCTL_CODE_LABELS[] = {
		[PIP2_IOCTL_CODE_ERASE_FILE]         = "Erase file",
		[PIP2_IOCTL_CODE_SEEK_FILE_POINTERS] = "Seek file pointers",
		[PIP2_IOCTL_CODE_AES_CONTROL]        = "AES control",
		[PIP2_IOCTL_CODE_FILE_STATS]         = "File stats",
		[PIP2_IOCTL_CODE_FILE_CRC]           = "File CRC"
};

static ChannelType active_channel_type = CHANNEL_TYPE_NONE;
static int i2c_dev_fd = -1;
static int i2c_bus;
static int i2c_addr;

static int _send_report_via_i2cdev(const ReportData* report);
static Poll_Status _get_report_from_i2cdev(ReportData* report,
		bool apply_timeout, long double timeout_val);
static int _verify_pip2_response(uint8_t seq, PIP2_Cmd_ID cmd_id,
		const PIP2_Rsp_Header* rsp);

int (*send_pip2_cmd_via_channel)(const ReportData* report);

Poll_Status (*get_pip2_rsp_via_channel)(ReportData* report, bool apply_timeout,
		long double timeout_val);

int do_pip2_command(ReportData* cmd, ReportData* rsp)
{
	const PIP2_Cmd_Header* cmd_header;
	const PIP2_Rsp_Header* rsp_header;
	size_t payload_len = 0;
	int rc;
	Poll_Status read_rc;
	ReportData rsp_report;

	cmd_header = (PIP2_Cmd_Header*) cmd->data;

	if (cmd_header->cmd_reg_lsb != PIP2_CMD_REG_LSB
			|| cmd_header->cmd_reg_msb != PIP2_CMD_REG_MSB) {
		output(ERROR,
				"%s: PIP2 command has incorrect CMD Register. Got 0x%02X%02X, "
				"expected 0x%02X%02X.\n",
				__func__, cmd_header->cmd_reg_lsb, cmd_header->cmd_reg_msb,
				PIP2_CMD_REG_LSB, PIP2_CMD_REG_MSB);
		return EXIT_FAILURE;
	}

	rsp_report.data = NULL;
	rsp_report.max_len = PIP2_PAYLOAD_MAX_LEN;
	rsp_report.data = (uint8_t*) calloc(rsp_report.max_len, sizeof(uint8_t));
	if (rsp_report.data == NULL) {
		output(ERROR, "%s: Memory allocation failed. %s [%d].\n", __func__,
				strerror(errno), errno);
		return EXIT_FAILURE;
	}

	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_PIP2,
			PIP2_CMD_NAMES[cmd_header->cmd_id], REPORT_TYPE_COMMAND, cmd);
	rc = send_pip2_cmd_via_channel(cmd);
	if (rc != EXIT_SUCCESS) {
		goto RETURN;
	}

	if (cmd_header->cmd_id == PIP2_CMD_ID_FILE_IOCTL) {
		sleep(FILE_IOCTL_ERASE_DELAY_BETWEEN_CMD_AND_RSP);
	} else {
		sleep_ms(AVG_DELAY_BETWEEN_CMD_AND_RSP);
	}

	memset(rsp_report.data, 0, sizeof(rsp_report.max_len));
	read_rc = get_pip2_rsp_via_channel(&rsp_report, true,
			MAX_TIMEOUT_BETWEEN_CMD_AND_RSP);
	switch (read_rc) {
	case POLL_STATUS_GOT_DATA:
		break;
	case POLL_STATUS_TIMEOUT:
		output(ERROR,
				"%s: Timed-Out waiting for PIP2 %s Response.\n",
				__func__, PIP2_CMD_NAMES[cmd_header->cmd_id]);
		rc = EXIT_FAILURE;
		break;
	case POLL_STATUS_ERROR:
		output(ERROR,
				"%s: Unexpected error occurred while attempting to retrieve"
				" the PIP2 %s Response.\n",
				__func__, PIP2_CMD_NAMES[cmd_header->cmd_id]);
		rc = EXIT_FAILURE;
		break;
	default:
		output(ERROR,
				"%s: Unexpected 'Poll_Status' enum value (%d) for pending "
				"PIP2 %s Response.\n",
				__func__, read_rc, PIP2_CMD_NAMES[cmd_header->cmd_id]);
		rc = EXIT_FAILURE;
	}

	if (rsp_report.len < PIP2_RSP_MIN_LEN) {
		output(ERROR,
				"%s: The PIP2 %s Response is shorter than the min expected "
				"(%lu bytes) among those of all PIP2 Commands.\n",
				__func__, PIP2_CMD_NAMES[cmd_header->cmd_id], rsp_report.len);
		rc = EXIT_FAILURE;
	}

	if (rc == EXIT_SUCCESS) {
		rsp_header = (PIP2_Rsp_Header*) rsp_report.data;
		rc = _verify_pip2_response(rsp_header->seq, rsp_header->cmd_id,
				rsp_header);
	}

	if (rc != EXIT_SUCCESS) {
		goto RETURN;
	}

	payload_len = ((rsp_header->payload_len_msb << 8)
			| rsp_header->payload_len_lsb);
	output(DEBUG, "Payload Length: %u\n", payload_len);
	output_debug_report(REPORT_DIRECTION_INCOMING_FROM_DUT,
			REPORT_FORMAT_PIP2, PIP2_CMD_NAMES[rsp_header->cmd_id],
			REPORT_TYPE_RESPONSE, &rsp_report);

	if (rsp != NULL) {
		if (payload_len > rsp->max_len) {
			output(ERROR, "%s: The PIP2 response payload is larger (%u bytes) "
					"than the maximum size supported (%u bytes).\n",
					__func__, payload_len, rsp->max_len);
			rc = EXIT_FAILURE;
		} else if (rsp_report.len > payload_len) {
			output(ERROR, "%s: The PIP2 response is larger than specified in "
					"the payload length field (%u) than expected (%u bytes).\n",
					__func__, rsp_report.len, payload_len);
			rc = EXIT_FAILURE;
		} else {
			memcpy(rsp->data, rsp_report.data, rsp_report.len);
		}
	}

RETURN:
	free(rsp_report.data);
	return rc;
}

int do_pip2_file_close_cmd(uint8_t seq_num, uint8_t file_handle,
		PIP2_Rsp_Payload_FileClose* rsp)
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

	uint16_t cmd_payload_len = sizeof(PIP2_Cmd_Payload_FileClose) - 2;
	PIP2_Cmd_Payload_FileClose cmd_data = {
			.header = {
					.cmd_reg_lsb        = PIP2_CMD_REG_LSB,
					.cmd_reg_msb        = PIP2_CMD_REG_MSB,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP2_CMD_ID_FILE_CLOSE,
					.resp               = 0
			},
			.file_handle = file_handle,
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
			cmd.len - 4);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP2_Rsp_Payload_FileClose)
	};

	return do_pip2_command(&cmd, &_rsp);
}

int do_pip2_file_ioctl_erase_file_cmd(uint8_t seq_num, uint8_t file_handle,
		PIP2_Rsp_Payload_FileIOCTL_EraseFile* rsp)
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

	uint16_t cmd_payload_len = sizeof(PIP2_Cmd_Payload_FileIOCTL_EraseFile) - 2;
	PIP2_Cmd_Payload_FileIOCTL_EraseFile cmd_data = {
			.header = {
					.cmd_reg_lsb        = PIP2_CMD_REG_LSB,
					.cmd_reg_msb        = PIP2_CMD_REG_MSB,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP2_CMD_ID_FILE_IOCTL,
					.resp               = 0
			},
			.file_handle = file_handle,
			.ioctl_code  = (uint8_t) PIP2_IOCTL_CODE_ERASE_FILE
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
			cmd.len - 4);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP2_Rsp_Payload_FileIOCTL_EraseFile)
	};

	return do_pip2_command(&cmd, &_rsp);
}

int do_pip2_file_ioctl_file_crc_cmd(uint8_t seq_num, uint8_t file_handle,
		uint32_t offset, uint32_t length,
		PIP2_Rsp_Payload_FileIOCTL_FileCRC* rsp)
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

	uint16_t cmd_payload_len = sizeof(PIP2_Cmd_Payload_FileIOCTL_FileCRC) - 2;
	PIP2_Cmd_Payload_FileIOCTL_FileCRC cmd_data = {
			.header = {
					.cmd_reg_lsb        = PIP2_CMD_REG_LSB,
					.cmd_reg_msb        = PIP2_CMD_REG_MSB,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP2_CMD_ID_FILE_IOCTL,
					.resp               = 0
			},
			.file_handle = file_handle,
			.ioctl_code  = (uint8_t) PIP2_IOCTL_CODE_FILE_CRC,
			.offset = offset,
			.length = length
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
			cmd.len - 4);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP2_Rsp_Payload_FileIOCTL_FileCRC)
	};

	return do_pip2_command(&cmd, &_rsp);
}

int do_pip2_file_open_cmd(uint8_t seq_num, uint8_t file_num,
		PIP2_Rsp_Payload_FileOpen* rsp)
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

	uint16_t cmd_payload_len = sizeof(PIP2_Cmd_Payload_FileOpen) - 2;
	PIP2_Cmd_Payload_FileOpen cmd_data = {
			.header = {
					.cmd_reg_lsb        = PIP2_CMD_REG_LSB,
					.cmd_reg_msb        = PIP2_CMD_REG_MSB,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP2_CMD_ID_FILE_OPEN,
					.resp               = 0
			},
			.file_num = file_num,
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
			cmd.len - 4);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP2_Rsp_Payload_FileOpen)
	};

	return do_pip2_command(&cmd, &_rsp);
}

int do_pip2_file_read_cmd(uint8_t seq_num, uint8_t file_handle,
		uint16_t read_len, PIP2_Rsp_Payload_FileRead* rsp, size_t max_rsp_size)
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

	uint16_t cmd_payload_len = sizeof(PIP2_Cmd_Payload_FileRead) - 2;
	PIP2_Cmd_Payload_FileRead cmd_data = {
			.header = {
					.cmd_reg_lsb        = PIP2_CMD_REG_LSB,
					.cmd_reg_msb        = PIP2_CMD_REG_MSB,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP2_CMD_ID_FILE_READ,
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
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
			cmd.len - 4);
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

	rc = do_pip2_command(&cmd, &_rsp);
	if (_rsp.len < PIP2_RSP_MIN_LEN) {
		output(ERROR, "%s: PIP2 FILE_READ response is shorter than the min "
				"possible PIP2 reponse (% bytes).\n",
				__func__, PIP2_RSP_MIN_LEN);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	memcpy((void*) &rsp->header, (void*) _rsp.data, sizeof(PIP2_Rsp_Header));

	memcpy((void*) rsp->data, (void*) &_rsp.data[sizeof(PIP2_Rsp_Header)],
			read_len);

	rsp->footer.crc_msb = _rsp.data[_rsp.len - 2];
	rsp->footer.crc_lsb = _rsp.data[_rsp.len - 1];

	rsp_crc = calculate_crc16_ccitt(0xFFFF, _rsp.data, _rsp.len - 2);
	rsp_crc_msb = rsp_crc >> 8;
	rsp_crc_lsb = rsp_crc & 0xFF;

	if (rsp->footer.crc_msb != rsp_crc_msb
			|| rsp->footer.crc_lsb != rsp_crc_lsb) {
		output(ERROR,
				"Unexpected PIP2 Response CRC:\n"
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

int do_pip2_file_write_cmd(uint8_t seq_num, uint8_t file_handle, ByteData* data)
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

	cmd.max_len = PIP2_FILE_WRITE_CMD_MAX_LEN;
	max_data_per_cmd_len = cmd.max_len - PIP2_FILE_WRITE_CMD_WITHOUT_DATA_LEN;
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
		PIP2_Rsp_Payload_FileWrite rsp;
		ReportData _rsp = {
				.data        = (uint8_t*) &rsp,
				.len         = 0,
				.index       = 0,
				.num_records = 0,
				.max_len     = sizeof(PIP2_Rsp_Payload_FileWrite)
		};

		data_part_len = ((remaining_data_len > max_data_per_cmd_len)
				? max_data_per_cmd_len : remaining_data_len);

		cmd.len = data_part_len + PIP2_FILE_WRITE_CMD_WITHOUT_DATA_LEN;
		if (cmd.len > cmd.max_len) {
			output(ERROR,
					"%s: PIP2 Command length (%u bytes) is too large per the "
					"ROM Bootloader spec (%u bytes).\n",
					__func__, cmd.len, cmd.max_len);
			rc = EXIT_FAILURE;
			error_occurred = true;
		}

		if (!error_occurred) {
			remaining_data_len -= data_part_len;

			PIP2_Cmd_Payload_FileWrite* cmd_data =
					(PIP2_Cmd_Payload_FileWrite*) cmd.data;
			uint16_t cmd_payload_len = (uint16_t) cmd.len - 2;

			cmd_data->header.cmd_reg_lsb = PIP2_CMD_REG_LSB;
			cmd_data->header.cmd_reg_msb = PIP2_CMD_REG_MSB;
			cmd_data->header.payload_len_lsb = cmd_payload_len & 0xFF;
			cmd_data->header.payload_len_msb = cmd_payload_len >> 8;
			cmd_data->header.seq = seq_num;
			cmd_data->header.tag = TAG_BIT;
			cmd_data->header.reserved_section_1 = 0;
			cmd_data->header.cmd_id = (uint8_t) PIP2_CMD_ID_FILE_WRITE;
			cmd_data->header.resp = 0;
			cmd_data->file_handle = file_handle;

			memcpy((void*) &cmd.data[7],
					(void*) &data->data[data_part_start_index], data_part_len);
			cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
					cmd.len - 4);
			cmd.data[cmd.len - 2] = cmd_crc >> 8;
			cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

			rc = do_pip2_command(&cmd, &_rsp);
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

int do_pip2_reset_cmd(uint8_t seq_num)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;

	uint16_t cmd_payload_len = sizeof(PIP2_Cmd_Payload_Reset) - 2;
	PIP2_Cmd_Payload_Reset cmd_data = {
			.header = {
					.cmd_reg_lsb        = PIP2_CMD_REG_LSB,
					.cmd_reg_msb        = PIP2_CMD_REG_MSB,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP2_CMD_ID_RESET,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
			cmd.len - 4);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	output_debug_report(REPORT_DIRECTION_OUTGOING_TO_DUT, REPORT_FORMAT_PIP2,
			PIP2_CMD_NAMES[PIP2_CMD_ID_RESET], REPORT_TYPE_COMMAND, &cmd);
	rc = send_pip2_cmd_via_channel(&cmd);

	output(DEBUG, "Waiting %u seconds to give the DUT enough time to reset.\n",
			DELAY_FOR_RESET);
	sleep(DELAY_FOR_RESET);

	output(DEBUG, "%s: Leaving.\n", __func__);
	return rc;
}

int do_pip2_status_cmd(uint8_t seq_num, PIP2_Rsp_Payload_Status* rsp)
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

	uint16_t cmd_payload_len = sizeof(PIP2_Cmd_Payload_Status) - 2;
	PIP2_Cmd_Payload_Status cmd_data = {
			.header = {
					.cmd_reg_lsb        = PIP2_CMD_REG_LSB,
					.cmd_reg_msb        = PIP2_CMD_REG_MSB,
					.payload_len_lsb    = cmd_payload_len & 0xFF,
					.payload_len_msb    = cmd_payload_len >> 8,
					.seq                = seq_num,
					.tag                = TAG_BIT,
					.reserved_section_1 = 0,
					.cmd_id             = (uint8_t) PIP2_CMD_ID_STATUS,
					.resp               = 0
			}
	};
	ReportData cmd = {
			.data = (uint8_t*) &cmd_data,
			.len  = sizeof(cmd_data)
	};
	uint16_t cmd_crc = calculate_crc16_ccitt(0xFFFF, &(cmd.data[2]),
			cmd.len - 4);
	cmd.data[cmd.len - 2] = cmd_crc >> 8;
	cmd.data[cmd.len - 1] = cmd_crc & 0xFF;

	ReportData _rsp = {
			.data        = (uint8_t*) rsp,
			.len         = 0,
			.index       = 0,
			.num_records = 0,
			.max_len     = sizeof(PIP2_Rsp_Payload_Status)
	};

	return do_pip2_command(&cmd, &_rsp);
}

bool is_pip2_api_active()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	return active_channel_type != CHANNEL_TYPE_NONE;
}

int setup_pip2_api(ChannelType channel_type, int i2c_bus_arg, int i2c_addr_arg)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (active_channel_type != CHANNEL_TYPE_NONE
			&& active_channel_type != channel_type) {
		output(ERROR, "%s: The API is already configured to use the channel.\n",
				CHANNEL_TYPE_NAMES[channel_type]);
		return EXIT_FAILURE;
	} else if (active_channel_type != CHANNEL_TYPE_NONE) {
		output(DEBUG, "%s: The API is already configured to use the channel.\n",
						CHANNEL_TYPE_NAMES[channel_type]);
		return EXIT_SUCCESS;
	}

	switch (channel_type) { 
	case CHANNEL_TYPE_I2CDEV:
		i2c_bus = i2c_bus_arg;
		i2c_addr = i2c_addr_arg;
		send_pip2_cmd_via_channel = _send_report_via_i2cdev;
		get_pip2_rsp_via_channel = _get_report_from_i2cdev;
		break;
	default:
		output(ERROR, "%s: Given an invalid/unsupported channel type.\n",
				__func__);
		return EXIT_FAILURE;
	}

	active_channel_type = channel_type;
	output(DEBUG, "Using the %s channel type.\n",
				CHANNEL_TYPE_NAMES[channel_type]);

	return EXIT_SUCCESS;
}

int teardown_pip2_api()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	switch (active_channel_type) {
	case CHANNEL_TYPE_NONE:
		output(DEBUG, "API is already inactive.\n");
		break;
	case CHANNEL_TYPE_I2CDEV:
		active_channel_type = CHANNEL_TYPE_NONE;
		break;
	default:
		output(ERROR, "%s: Given an invalid/unsupported channel type.\n",
				__func__);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

static int _send_report_via_i2cdev(const ReportData* report)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	char filename[20] = {0};

	i2c_dev_fd = open_i2c_dev(i2c_bus, filename, sizeof(filename), 0);
	if (i2c_dev_fd < 0) {
		output(ERROR, "%s: Failed to open the i2c-dev sysfs node for I2C bus "
				"%d. %s [%d].\n",
				__func__, i2c_bus, strerror(errno), errno);
		return EXIT_FAILURE;
	}
	errno = 0;

	if (set_slave_addr(i2c_dev_fd, i2c_addr, 1) != 0) {
		output(ERROR,
				"%s: Failed to set I2C slave device 0x%02X. %s [%d].\n",
				__func__, i2c_addr, strerror(errno), errno);
		close(i2c_dev_fd);
		return EXIT_FAILURE;
	}

	ssize_t num_bytes_written = write(i2c_dev_fd, report->data, report->len);
	if (errno != 0) {
		output(ERROR, "%s: Failed to write report to %s. %s [%d].\n", __func__,
				filename, strerror(errno), errno);
	} else if (num_bytes_written != report->len) {
		output(ERROR,
				"%s: Number of bytes written (%d) does not match the expected "
				"cmd size (%lu).\n",
				__func__, num_bytes_written, report->len);
	} else {
		rc = EXIT_SUCCESS;
	}

	return rc;
}

static Poll_Status _get_report_from_i2cdev(ReportData* report,
		bool apply_timeout, long double timeout_val 
		)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	Poll_Status rc = POLL_STATUS_ERROR;
	uint8_t rsp_len_bytes[2] = {0};

	ssize_t num_bytes_read = read(i2c_dev_fd, rsp_len_bytes, 2);
	if (errno != 0) {
		output(ERROR, "%s: Failed to read the report length. %s [%d].\n",
				__func__, strerror(errno), errno);
		rc = POLL_STATUS_ERROR;
		goto RETURN;
	} else if (num_bytes_read != 2) {
		output(ERROR,
				"%s: Failed to read the report length. Read %d bytes but "
				"expected %lu.\n",
				__func__, num_bytes_read, 2);
		rc = POLL_STATUS_ERROR;
		goto RETURN;
	}
	report->len = (size_t) ((rsp_len_bytes[1] << 8) | rsp_len_bytes[0]);

	sleep_ms(AVG_DELAY_BETWEEN_CMD_AND_RSP);

	num_bytes_read = read(i2c_dev_fd, report->data, report->len);
	if (errno != 0) {
		output(ERROR, "%s: Failed to read the full report. %s [%d].\n",
				__func__, strerror(errno), errno);
		rc = POLL_STATUS_ERROR;
	} else if (num_bytes_read != report->len) {
		output(ERROR,
				"%s: Failed to read the full report. Read %d bytes but expected"
				"%lu.\n",
				__func__, num_bytes_read, report->len);
		rc = POLL_STATUS_ERROR;
	} else {
		rc = POLL_STATUS_GOT_DATA;
	}

RETURN:
	close(i2c_dev_fd);
	return rc;
}

static int _verify_pip2_response(uint8_t seq, PIP2_Cmd_ID cmd_id,
		const PIP2_Rsp_Header* rsp)
{
	if (cmd_id != rsp->cmd_id) {
		output(ERROR,
				"%s: Got response for unexpected %s (0x%02X) command.\n",
				__func__, PIP2_CMD_NAMES[rsp->cmd_id], rsp->cmd_id);
		return EXIT_FAILURE;
	}

	if (seq != rsp->seq) {
		output(ERROR,
				"%s: The Command SEQ (%u) and Response SEQ (%u) do not match."
				"\n",
				__func__, seq, rsp->seq);
		return EXIT_FAILURE;
	}

	if (rsp->resp != 1) {
		output(ERROR,
				"%s: RESP bit must always be set for PIP2 responses.\n",
				__func__);
		return EXIT_FAILURE;
	}

	if (rsp->status_code != PIP2_STATUS_CODE_SUCCESS) {
		output(ERROR, "%s: PIP2 %s (0x%02X) command failed: %s.\n",
				__func__, PIP2_CMD_NAMES[cmd_id], cmd_id,
				PIP2_STATUS_CODE_LABELS[rsp->status_code]);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
