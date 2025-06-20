/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "dut_utils.h"

#define FW_SELF_TEST_OUTPUT_FORMAT_U8    1
#define FW_SELF_TEST_OUTPUT_FORMAT_U16   2

#define EXIT_ALREADY_IN_REQUESTED_STATE 2

#define MAX_NUM_OF_BYTES_PER_SENSOR  8

char* FW_LOADER_NAMES[] = {
	[FLASH_LOADER_NONE]                     = "No active/valid flash loader",
	[FLASH_LOADER_TP_PROGRAMMER_IMAGE]      = "TP Programmer Image",
	[FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE] = "AUX MCU Programmer Image",
	[FLASH_LOADER_PIP2_ROM_BL]              = "PIP2 ROM-Bootloader",
};

static DUT_State active_dut_state = DUT_STATE_DEFAULT;
static Flash_Loader active_flash_loader = FLASH_LOADER_NONE;
static struct timeval aux_mcu_active_start_time;

static int _enter_flash_loader(const Flash_Loader_Options* options);
static int _erase_config_file(uint8_t config_file_num);
static int _exit_flash_loader();
static int _flash_file_close(uint8_t file_handle);
static int _flash_file_erase(uint8_t file_handle);
static int _flash_file_open(uint8_t file_num, uint8_t* file_handle);
static int _flash_file_write(uint8_t file_handle, ByteData* data);
static DUT_State _get_dut_state_from_fw_sys_mode(PIP3_App_Sys_Mode sys_mode);
static int _set_dut_state_aux_mcu_fw_programmer_img();
static int _set_dut_state_aux_mcu_fw_utility_img();
static int _set_dut_state_tp_bl_exec();
static int _set_dut_state_tp_fw_exec();
static int _set_dut_state_tp_fw_scanning();
static int _set_dut_state_tp_fw_test();
static int _set_dut_state_tp_programmer_img();
static int _verify_active_processor(PIP3_Processor_ID expected_processor,
		long double timeout_seconds);
static int _verify_fw_category(PIP3_FW_Category_ID expected_fw_category_id);

#define CALIBRATION_MODE_FULL_CALIBRATION (0x00)
#define CALIBRATION_PARAM_DATA_IGNORED    (0x00)

int calibrate_dut()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;

	rc = set_dut_state(DUT_STATE_TP_FW_TEST);
	if (rc != EXIT_SUCCESS)
	{
		return rc;
	}

	rc = do_pip3_calibrate_cmd(0x00, CALIBRATION_MODE_FULL_CALIBRATION,
			CALIBRATION_PARAM_DATA_IGNORED, CALIBRATION_PARAM_DATA_IGNORED,
			CALIBRATION_PARAM_DATA_IGNORED);

	if (set_dut_state(DUT_STATE_TP_FW_SCANNING) != EXIT_SUCCESS)
	{
		rc = EXIT_FAILURE;
	}

	return rc;
}

int do_dut_fw_self_test(PIP3_Self_Test_ID self_test_id,
		int output_format_id, ByteData* cmd_params, bool signed_data,
		bool length_known, FW_Self_Test_Results* results)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	PIP3_Rsp_Payload_GetSelfTestResults get_self_test_results_rsp;
	PIP3_Rsp_Payload_RunSelfTest run_self_test_rsp;
	size_t max_rsp_len;
	size_t bytes_read;
	size_t expected_bytes_read;
	int byte_index = 0;
	int val_index = 0;
	bool scanning_suspended = false;

	if (results == NULL || results->data == NULL) {
		output(ERROR, "%s: NULL argument provided.\n", __func__);
		return EXIT_FAILURE;
	} else if (output_format_id != FW_SELF_TEST_OUTPUT_FORMAT_U8
			&& output_format_id != FW_SELF_TEST_OUTPUT_FORMAT_U16) {
		output(ERROR,
				"%s: PtMfg only supports output format 1 or 2 (%d given).\n",
				__func__, output_format_id);
		return EXIT_FAILURE;
	}

	max_rsp_len = (results->max_len * MAX_NUM_OF_BYTES_PER_SENSOR)
			+ sizeof(PIP3_Rsp_Payload_GetSelfTestResults);

	get_self_test_results_rsp.data = (uint8_t*) calloc(max_rsp_len, 1);
	if (get_self_test_results_rsp.data == NULL) {
		output(ERROR, "%s: Memory allocation failed.\n", __func__);
		return EXIT_FAILURE;
	}

	rc = set_dut_state(DUT_STATE_TP_FW_TEST);
	if (rc != EXIT_SUCCESS) {
		goto RETURN;
	}
	scanning_suspended = true;

	if (cmd_params != NULL && cmd_params->len > 0
			&& EXIT_SUCCESS != do_pip3_load_self_test_param_cmd(
									0x00, (uint8_t) self_test_id, cmd_params)) {
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	rc = do_pip3_run_self_test_cmd(0x00, (uint8_t) self_test_id,
			&run_self_test_rsp);
	if (rc != EXIT_SUCCESS) {
		goto RETURN;
	}

	rc = do_pip3_get_self_test_results_cmd(0x00, (uint8_t) self_test_id,
			&get_self_test_results_rsp, max_rsp_len);
	if (rc != EXIT_SUCCESS) {
		goto RETURN;
	}

	bytes_read = ((get_self_test_results_rsp.arl_msb << 8)
			| get_self_test_results_rsp.arl_lsb);

	expected_bytes_read = results->max_len;

	if (output_format_id == FW_SELF_TEST_OUTPUT_FORMAT_U16) {
		output(DEBUG, "The output format is with one 2-byte word per value.\n");
		expected_bytes_read *= 2;
	}

	if (PIP3_DATA_FORMAT_ID_2BYTE_UNSIGNED_PLUS_EXTRA
			== get_self_test_results_rsp.data_format_id) {
		output(DEBUG,
				"The results for the %s self-test include an extra 2-byte value"
				" representing the average calculated at the most recent "
				"calibration.\n",
				PIP3_SELF_TEST_NAMES[self_test_id]);
		expected_bytes_read += 2;
	}

	if (length_known && bytes_read != expected_bytes_read) {
		output(ERROR,
			"%s: Unexpected response length. Expected %d values, "
			"received %d.\n",
			__func__, expected_bytes_read, bytes_read);
		rc = EXIT_FAILURE;
		goto RETURN;
	}

	while (byte_index < bytes_read) {
		if (output_format_id == FW_SELF_TEST_OUTPUT_FORMAT_U8) {
			results->data[val_index] =
					get_self_test_results_rsp.data[byte_index];
			byte_index++;
		} else {
			results->data[val_index] =
					*((uint16_t*) &get_self_test_results_rsp.data[byte_index]);
			if (signed_data) {
				output(DEBUG,
						"Extend from 2-byte to 4-byte values for signed data.\n"
						);
				results->data[val_index] = (int16_t) results->data[val_index];
			}
			byte_index += 2;
		}
		val_index++;
		results->read_len++;
	}

RETURN:
	if (scanning_suspended && EXIT_SUCCESS
			!= set_dut_state(DUT_STATE_TP_FW_SCANNING)) {
		rc = EXIT_FAILURE;
	}

	free(get_self_test_results_rsp.data);
	return rc;
}

int read_dut_fw_bin_header(FW_Bin_Header* bin_header)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int cmd_rc = EXIT_FAILURE;
	bool file_open = false;
	PIP3_Rsp_Payload_FileOpen file_open_rsp;
	PIP3_Rsp_Payload_FileRead file_read_rsp;
	bool in_secondary_img = false;
	size_t max_rsp_len;
	int rc = EXIT_FAILURE;

	cmd_rc = set_dut_state(DUT_STATE_TP_FW_PROGRAMMER_IMAGE);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	} else {
		in_secondary_img = true;
	}

	cmd_rc = do_pip3_file_open_cmd(0x00, PRIMARY_FW_BIN_FILE_NUM,
			&file_open_rsp);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	} else {
		output(DEBUG, "Opened the Primary FW bin file.\n");
		file_open = true;
	}

	file_read_rsp.data = (uint8_t*) bin_header;
	max_rsp_len = FW_BIN_HEADER_SIZE + sizeof(PIP3_Rsp_Payload_FileRead);
	cmd_rc = do_pip3_file_read_cmd(0x00, file_open_rsp.file_handle,
			FW_BIN_HEADER_SIZE, &file_read_rsp, max_rsp_len);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	}

	rc = EXIT_SUCCESS;

RETURN:
	if (file_open) {
		PIP3_Rsp_Payload_FileClose file_close_rsp;
		cmd_rc = do_pip3_file_close_cmd(0x00, file_open_rsp.file_handle,
				&file_close_rsp);
		if (cmd_rc != EXIT_SUCCESS) rc = cmd_rc;
	}

	if (in_secondary_img) {
		cmd_rc = set_dut_state(DUT_STATE_TP_FW_SCANNING);
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	return rc;
}

int set_dut_state(DUT_State target_state)
{
	switch (target_state) {
	case DUT_STATE_TP_FW_BOOT:
		goto RETURN_NOT_SUPPORTED;
	case DUT_STATE_TP_FW_SCANNING:
		return _set_dut_state_tp_fw_scanning();
	case DUT_STATE_TP_FW_DEEP_SLEEP:
		goto RETURN_NOT_SUPPORTED;
	case DUT_STATE_TP_FW_TEST:
		return _set_dut_state_tp_fw_test();
	case DUT_STATE_TP_FW_DEEP_STANDBY:
		goto RETURN_NOT_SUPPORTED;
	case DUT_STATE_TP_FW_PROGRAMMER_IMAGE:
		return _set_dut_state_tp_programmer_img();
	case DUT_STATE_TP_FW_SYS_MODE_ANY:
		return _set_dut_state_tp_fw_exec();
	case DUT_STATE_TP_BL:
		return _set_dut_state_tp_bl_exec();
	case DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE:
		if (get_dut_driver() == DUT_DRIVER_TTDL) {
			goto RETURN_NOT_SUPPORTED;
		}
		return _set_dut_state_aux_mcu_fw_utility_img();
	case DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE:
		if (get_dut_driver() == DUT_DRIVER_TTDL) {
			active_flash_loader = FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE;
			active_dut_state = DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE;
			return EXIT_SUCCESS;
		} else {
			return _set_dut_state_aux_mcu_fw_programmer_img();
		}
	case DUT_STATE_DEFAULT:
		return EXIT_SUCCESS;
	default:
		output(ERROR,
				"%s: Unrecognized target 'DUT_State' enum value: %d.\n",
				__func__, target_state);
		return EXIT_FAILURE;
	}

RETURN_NOT_SUPPORTED:
	output(ERROR, "%s: This DUT state is not currently support for "
			"drivers/channels other than TTDL.\n",
			__func__, DUT_STATE_LABELS[target_state]);
	return EXIT_FAILURE;
}

int write_image_to_dut_flash_file(uint8_t file_num, ByteData* image,
		const ByteData* file_nums_to_erase,
		const Flash_Loader_Options* loader_options)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	int cmd_rc = EXIT_FAILURE;
	uint8_t file_handle;
	bool file_open = false;
	bool switched_state = false;

	if (file_nums_to_erase != NULL && file_nums_to_erase->data == NULL) {
		output(ERROR,
				"%s: The provided 'file_nums_to_erase->data' struct-member is "
				"NULL.\n",
				__func__);
		return EXIT_FAILURE;
	} else if (file_nums_to_erase != NULL
			&& file_nums_to_erase->len > MAX_NUM_OF_FILES_TO_ERASE) {
		output(ERROR,
				"%s: Provided beyond the maximum supported number of files to "
				"erase. Got %u, expected %u or less.\n",
				__func__, file_nums_to_erase->len, MAX_NUM_OF_FILES_TO_ERASE);
		return EXIT_FAILURE;
	}

	cmd_rc = _enter_flash_loader(loader_options);
	if (cmd_rc == EXIT_FAILURE) {
		rc = cmd_rc;
		goto RETURN;
	} else if (cmd_rc == EXIT_SUCCESS) {
		switched_state = true;
	}

	cmd_rc = _flash_file_open(file_num, &file_handle);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	} else {
		output(DEBUG, "Opened the flash file ID %u.\n", file_num);
		file_open = true;
	}

	cmd_rc = _flash_file_erase(file_handle);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	}

	for (int i = 0; file_nums_to_erase != NULL && i < file_nums_to_erase->len;
			i++) {
		cmd_rc = _erase_config_file(file_nums_to_erase->data[i]);
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
			goto RETURN;
		}
	}

	rc = _flash_file_write(file_handle, image);

RETURN:
	if (file_open) {
		cmd_rc = _flash_file_close(file_handle);
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	if (switched_state) {
		cmd_rc = _exit_flash_loader();
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	return rc;
}

static int _enter_flash_loader(const Flash_Loader_Options* options)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (active_flash_loader != FLASH_LOADER_NONE) {
		for (int i = 0; i < NUM_OF_FLASH_LOADERS; i++) {
			if (active_flash_loader == options->list[i]) {
				output(DEBUG, "The %s is already active.\n",
						FW_LOADER_NAMES[active_flash_loader]);
				return EXIT_ALREADY_IN_REQUESTED_STATE;
			} else if (FLASH_LOADER_NONE == options->list[i]) {
				break;
			}
		}

		output(ERROR, "%s: The %s is already active.\n", __func__,
				FW_LOADER_NAMES[active_flash_loader]);
		return EXIT_FAILURE;
	}

	int cmd_rc = EXIT_FAILURE;

	for (int i = 0;
			i < NUM_OF_FLASH_LOADERS && options->list[i] != FLASH_LOADER_NONE;
			i++) {
		switch (options->list[i]) {
		case FLASH_LOADER_TP_PROGRAMMER_IMAGE:
			cmd_rc = set_dut_state(DUT_STATE_TP_FW_PROGRAMMER_IMAGE);
			break;
		case FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE:
			cmd_rc = set_dut_state(DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE);
			break;
		case FLASH_LOADER_PIP2_ROM_BL:
			cmd_rc = set_dut_state(DUT_STATE_TP_BL);
			break;
		default:
			output(ERROR,
					"%s: Unexpected/unsupported 'Flash_Loader' enum value (%d)."
					"\n",
					__func__, options->list[i]);
			active_flash_loader = FLASH_LOADER_NONE;
			return EXIT_FAILURE;
		}

		if (cmd_rc != EXIT_FAILURE) {
			active_flash_loader = options->list[i];
			output(DEBUG,
					"Activated the %s for reading from/writing to flash.\n",
					FW_LOADER_NAMES[active_flash_loader]);
			return cmd_rc;
		}
	}

	output(ERROR, "%s: None valid firmware loader available or specified.\n",
			__func__);
	return EXIT_FAILURE;
}

static int _erase_config_file(uint8_t config_file_num)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int cmd_rc = EXIT_FAILURE;
	int rc = EXIT_FAILURE;
	bool file_open = false;
	uint8_t file_handle;

	if (config_file_num == 0x00) {
		output(DEBUG, "The Config file will not be erased.\n");
		rc = EXIT_SUCCESS;
		goto RETURN;
	}

	output(DEBUG, "Erasing the Config File (file num = 0x%02X).\n",
			config_file_num);

	cmd_rc = _flash_file_open(config_file_num, &file_handle);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	} else {
		output(DEBUG, "Opened the flash file ID %u.\n", config_file_num);
		file_open = true;
	}

	cmd_rc = _flash_file_erase(file_handle);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	}

	rc = EXIT_SUCCESS;

RETURN:
	if (file_open) {
		cmd_rc = _flash_file_close(file_handle);
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	return rc;
}

static int _exit_flash_loader()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (active_flash_loader == FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE) {
		if (get_dut_driver() == DUT_DRIVER_TTDL) {

			active_flash_loader = FLASH_LOADER_NONE;
			return EXIT_SUCCESS;
		} else {
			return set_dut_state(DUT_STATE_TP_FW_SYS_MODE_ANY);
		}
	}

	Flash_Loader initial_loader = active_flash_loader;

	if (FLASH_LOADER_PIP2_ROM_BL == active_flash_loader) {
		if (!is_pip3_api_active()) {
			output(WARNING,
				"Staying in the PIP2 ROM-BL because the PIP3 API is inactive.\n"
					);
			return EXIT_SUCCESS;
		} else if (EXIT_SUCCESS
				!= set_dut_state(DUT_STATE_TP_FW_SYS_MODE_ANY)) {
			return EXIT_FAILURE;
		}
	}

	if (EXIT_SUCCESS != set_dut_state(DUT_STATE_TP_FW_SCANNING)) {
		output(DEBUG,
				"Exited the ROM-BL but unable to enter Scanning mode.\n");
	}

	if (initial_loader == FLASH_LOADER_TP_PROGRAMMER_IMAGE
			&& active_flash_loader == FLASH_LOADER_TP_PROGRAMMER_IMAGE) {
		output(WARNING,
				"Stuck in the Touch Processor's Programmer image. Please update"
				" the Touch Processors Primary Touch firmware image.\n");
	}

	return EXIT_SUCCESS;
}

static int _flash_file_close(uint8_t file_handle)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;

	if (FLASH_LOADER_NONE == active_flash_loader) {
		output(ERROR, "%s: %s.\n",
				__func__, FW_LOADER_NAMES[active_flash_loader]);
		rc = EXIT_FAILURE;
	} else if (FLASH_LOADER_TP_PROGRAMMER_IMAGE == active_flash_loader
			|| FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE == active_flash_loader) {
		PIP3_Rsp_Payload_FileClose file_close_rsp;
		rc = do_pip3_file_close_cmd(0x00, file_handle, &file_close_rsp);
	} else if (FLASH_LOADER_PIP2_ROM_BL == active_flash_loader) {
		PIP2_Rsp_Payload_FileClose file_close_rsp;
		rc = do_pip2_file_close_cmd(0x00, file_handle, &file_close_rsp);
	} else {
		output(ERROR,
				"%s: Unexpected/unsupported 'Flash_Loader' enum value (%d).\n",
				__func__, active_flash_loader);
		rc = EXIT_FAILURE;
	}

	return rc;
}

static int _flash_file_erase(uint8_t file_handle)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;

	if (FLASH_LOADER_NONE == active_flash_loader) {
		output(ERROR, "%s: %s.\n",
				__func__, FW_LOADER_NAMES[active_flash_loader]);
		rc = EXIT_FAILURE;
	} else if (FLASH_LOADER_TP_PROGRAMMER_IMAGE == active_flash_loader
			|| FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE == active_flash_loader) {
		PIP3_Rsp_Payload_FileIOCTL_EraseFile file_ioctl_erase_rsp;
		rc = do_pip3_file_ioctl_erase_file_cmd(0x00, file_handle,
				&file_ioctl_erase_rsp);
	} else if (FLASH_LOADER_PIP2_ROM_BL == active_flash_loader) {
		PIP2_Rsp_Payload_FileIOCTL_EraseFile file_ioctl_erase_rsp;
		rc = do_pip2_file_ioctl_erase_file_cmd(0x00, file_handle,
				&file_ioctl_erase_rsp);
	} else {
		output(ERROR,
				"%s: Unexpected/unsupported 'Flash_Loader' enum value (%d).\n",
				__func__, active_flash_loader);
		rc = EXIT_FAILURE;
	}

	return rc;
}

static int _flash_file_open(uint8_t file_num, uint8_t* file_handle)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;

	if (FLASH_LOADER_NONE == active_flash_loader) {
		output(ERROR, "%s: %s.\n",
				__func__, FW_LOADER_NAMES[active_flash_loader]);
		rc = EXIT_FAILURE;
	} else if (FLASH_LOADER_TP_PROGRAMMER_IMAGE == active_flash_loader
			|| FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE == active_flash_loader) {
		PIP3_Rsp_Payload_FileOpen file_open_rsp;
		rc = do_pip3_file_open_cmd(0x00, file_num, &file_open_rsp);
		*file_handle = file_open_rsp.file_handle;
	} else if (FLASH_LOADER_PIP2_ROM_BL == active_flash_loader) {
		PIP2_Rsp_Payload_FileOpen file_open_rsp;
		rc = do_pip2_file_open_cmd(0x00, file_num, &file_open_rsp);
		*file_handle = file_open_rsp.file_handle;
	} else {
		output(ERROR,
				"%s: Unexpected/unsupported 'Flash_Loader' enum value (%d).\n",
				__func__, active_flash_loader);
		rc = EXIT_FAILURE;
	}

	return rc;
}

static int _flash_file_write(uint8_t file_handle, ByteData* data)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;

	if (FLASH_LOADER_NONE == active_flash_loader) {
		output(ERROR, "%s: %s.\n",
				__func__, FW_LOADER_NAMES[active_flash_loader]);
		rc = EXIT_FAILURE;
	} else if (FLASH_LOADER_TP_PROGRAMMER_IMAGE == active_flash_loader
			|| FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE == active_flash_loader) {
		rc = do_pip3_file_write_cmd(0x00, file_handle, data);
	} else if (FLASH_LOADER_PIP2_ROM_BL == active_flash_loader) {
		rc = do_pip2_file_write_cmd(0x00, file_handle, data);
	} else {
		output(ERROR,
				"%s: Unexpected/unsupported 'Flash_Loader' enum value (%d).\n",
				__func__, active_flash_loader);
		rc = EXIT_FAILURE;
	}

	return rc;
}

int flash_file_crc(uint8_t file_num, uint32_t offset, uint32_t length, uint16_t* crc)
{
	int rc = EXIT_FAILURE;
	int cmd_rc = EXIT_FAILURE;
	bool file_open = false;
	bool switched_state = false;
	uint8_t file_handle;

	output(DEBUG, "%s: Starting.\n", __func__);

	Flash_Loader_Options loader_options;
	loader_options.list[0] = FLASH_LOADER_TP_PROGRAMMER_IMAGE;
	loader_options.list[1] = FLASH_LOADER_NONE;
	cmd_rc = _enter_flash_loader(&loader_options);
	if (cmd_rc == EXIT_FAILURE) {
		rc = cmd_rc;
		goto RETURN;
	} else if (cmd_rc == EXIT_SUCCESS) {
		switched_state = true;
	}

	cmd_rc = _flash_file_open(file_num, &file_handle);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	} else {
		output(DEBUG, "Opened the flash file ID %u.\n", file_num);
		file_open = true;
	}

	if (FLASH_LOADER_NONE == active_flash_loader) {
		output(ERROR, "%s: %s.\n",
				__func__, FW_LOADER_NAMES[active_flash_loader]);
		rc = EXIT_FAILURE;
	} else if (FLASH_LOADER_TP_PROGRAMMER_IMAGE == active_flash_loader
			|| FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE == active_flash_loader) {
		PIP3_Rsp_Payload_FileIOCTL_FileCRC rsp;
		rc = do_pip3_file_ioctl_file_crc_cmd(0x00, file_handle, offset, length, &rsp);
		*crc = rsp.crc_lsb + (rsp.crc_msb << 8);
	} else if (FLASH_LOADER_PIP2_ROM_BL == active_flash_loader) {
		PIP2_Rsp_Payload_FileIOCTL_FileCRC rsp;
		rc = do_pip2_file_ioctl_file_crc_cmd(0x00, file_handle, offset, length, &rsp);
		*crc = rsp.crc_lsb + (rsp.crc_msb << 8);
	} else {
		output(ERROR,
				"%s: Unexpected/unsupported 'Flash_Loader' enum value (%d).\n",
				__func__, active_flash_loader);
		rc = EXIT_FAILURE;
	}

RETURN:
	if (file_open) {
		cmd_rc = _flash_file_close(file_handle);
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	if (switched_state) {
		cmd_rc = _exit_flash_loader();
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	return rc;
}

int flash_file_read(uint8_t file_num, uint16_t length, uint8_t* data)
{
	int rc = EXIT_FAILURE;
	int cmd_rc = EXIT_FAILURE;
	bool file_open = false;
	bool switched_state = false;
	uint8_t file_handle;

	output(DEBUG, "%s: Starting.\n", __func__);

	Flash_Loader_Options loader_options;
	loader_options.list[0] = FLASH_LOADER_TP_PROGRAMMER_IMAGE;
	loader_options.list[1] = FLASH_LOADER_NONE;
	cmd_rc = _enter_flash_loader(&loader_options);
	if (cmd_rc == EXIT_FAILURE) {
		rc = cmd_rc;
		goto RETURN;
	} else if (cmd_rc == EXIT_SUCCESS) {
		switched_state = true;
	}

	cmd_rc = _flash_file_open(file_num, &file_handle);
	if (cmd_rc != EXIT_SUCCESS) {
		rc = cmd_rc;
		goto RETURN;
	} else {
		output(DEBUG, "Opened the flash file ID %u.\n", file_num);
		file_open = true;
	}

	size_t max_rsp_len = length + sizeof(PIP3_Rsp_Payload_FileRead);
	if (FLASH_LOADER_NONE == active_flash_loader) {
		output(ERROR, "%s: %s.\n",
				__func__, FW_LOADER_NAMES[active_flash_loader]);
		rc = EXIT_FAILURE;
	} else if (FLASH_LOADER_TP_PROGRAMMER_IMAGE == active_flash_loader
			|| FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE == active_flash_loader) {
		PIP3_Rsp_Payload_FileRead rsp;
		rsp.data = data;
		rc = do_pip3_file_read_cmd(0x00, file_handle, length, &rsp, max_rsp_len);
	} else if (FLASH_LOADER_PIP2_ROM_BL == active_flash_loader) {
		PIP2_Rsp_Payload_FileRead rsp;
		rsp.data = data;
		rc = do_pip2_file_read_cmd(0x00, file_handle, length, &rsp, max_rsp_len);
	} else {
		output(ERROR,
				"%s: Unexpected/unsupported 'Flash_Loader' enum value (%d).\n",
				__func__, active_flash_loader);
		rc = EXIT_FAILURE;
	}

RETURN:
	if (file_open) {
		cmd_rc = _flash_file_close(file_handle);
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	if (switched_state) {
		cmd_rc = _exit_flash_loader();
		if (cmd_rc != EXIT_SUCCESS) {
			rc = cmd_rc;
		}
	}

	return rc;
}

int get_data_block(PIP3_Data_Block_ID data_block_id, ByteData* data_block, size_t max_len)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	int rc = EXIT_FAILURE;
	if (active_dut_state == DUT_STATE_TP_FW_SCANNING) {
		rc = set_dut_state(DUT_STATE_TP_FW_TEST);
		if (rc != EXIT_SUCCESS)
		{
			return rc;
		}
	}

	uint8_t tmp_data_block_data[0xffff];
	int data_block_size = 0;
	PIP3_Rsp_Payload_GetDataBlock tmp_rsp;
	bool first_received = false;
	do	{
		uint8_t tmp_buff[0xffff];
		tmp_rsp.data = tmp_buff;

		rc = do_pip3_get_data_block_cmd(0x00, data_block_size, 0xFFFF, data_block_id, &tmp_rsp, 0xffff);
		if (rc != EXIT_SUCCESS)
		{
			goto EXIT;
		}

		if ((data_block_size + tmp_rsp.actual_read_len) > max_len) {
			output(ERROR, "%s: Data block exceeds max size.\n", __func__);
			rc = EXIT_FAILURE;
			goto EXIT;
		}

		first_received = true;
		memcpy(&tmp_data_block_data[data_block_size], tmp_rsp.data, tmp_rsp.actual_read_len);
		data_block_size += tmp_rsp.actual_read_len;
	} while(tmp_rsp.header.more_data);

EXIT:
	if (!first_received)
	{
		return EXIT_FAILURE;
	}

	data_block->len = data_block_size;
	memcpy(data_block->data, tmp_data_block_data, data_block_size);

	return EXIT_SUCCESS;
}


static DUT_State _get_dut_state_from_fw_sys_mode(PIP3_App_Sys_Mode sys_mode)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	DUT_State dut_state;

	switch (sys_mode) {
	case PIP3_APP_SYS_MODE_BOOT:
		dut_state = DUT_STATE_TP_FW_BOOT;
		break;

	case PIP3_APP_SYS_MODE_SCANNING:
		dut_state = DUT_STATE_TP_FW_SCANNING;
		break;

	case PIP3_APP_SYS_MODE_DEEP_SLEEP:
		dut_state = DUT_STATE_TP_FW_DEEP_SLEEP;
		break;

	case PIP3_APP_SYS_MODE_TEST_CONFIG:
		dut_state = DUT_STATE_TP_FW_TEST;
		break;

	case PIP3_APP_SYS_MODE_DEEP_STANDBY:
		dut_state = DUT_STATE_TP_FW_DEEP_STANDBY;
		break;

	default:
		output(ERROR, "%s: Unrecognized FW system mode.\n", __func__);
		dut_state = DUT_STATE_INVALID;
	}

	return dut_state;
}

#define AUX_MCU_MAX_WAIT_TO_ACTIVATE_SECONDS 5

static int _set_dut_state_aux_mcu()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	errno = 0; 

	if (active_dut_state == DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE
			|| active_dut_state == DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE) {
		output(DEBUG, "Already setup for communication with AUX MCU.\n");
		return EXIT_SUCCESS;
	}

	if (EXIT_SUCCESS == _verify_active_processor(PIP3_PROCESSOR_ID_AUX_MCU, 0)) {
		output(DEBUG, "%s: DUT is already running from the AUX MCU.\n", __func__);
		return EXIT_SUCCESS;
	}

	output(INFO,
"\t Switching from the primary processor to the AUX MCU. This could take\n"
"\t several seconds.\n");
	gettimeofday(&aux_mcu_active_start_time, 0);

	if (EXIT_SUCCESS != do_pip3_switch_active_processor_cmd(0x00,
			PIP3_PROCESSOR_ID_AUX_MCU, get_aux_mcu_active_duration_seconds())) {
		return EXIT_FAILURE;
	}

	return _verify_active_processor(PIP3_PROCESSOR_ID_AUX_MCU,
			AUX_MCU_MAX_WAIT_TO_ACTIVATE_SECONDS);
}

static int _set_dut_state_aux_mcu_fw_programmer_img()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (active_dut_state == DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE) {
		output(DEBUG, "Already setup for communication with %s.\n",
				DUT_STATE_LABELS[DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE]);
		return EXIT_SUCCESS;
	}

	if (EXIT_SUCCESS != _set_dut_state_aux_mcu()) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS
			!= do_pip3_switch_image_cmd(0x00, PIP3_IMAGE_ID_SECONDARY)) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS
			!= _verify_fw_category(PIP3_FW_CATEGORY_ID_PROGRAMMER_FW)) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	active_dut_state = DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE;
	active_flash_loader = FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE;
	return EXIT_SUCCESS;
}

static int _set_dut_state_aux_mcu_fw_utility_img()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (active_dut_state == DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE) {
		output(DEBUG, "Already setup for communication with %s.\n",
				DUT_STATE_LABELS[DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE]);
		return EXIT_SUCCESS;
	}

	if (EXIT_SUCCESS != _set_dut_state_aux_mcu()) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS == _verify_fw_category(PIP3_FW_CATEGORY_ID_UTILITY_FW)) {
		active_dut_state = DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE;
		active_flash_loader = FLASH_LOADER_NONE;
		return EXIT_SUCCESS;
	}

	if (EXIT_SUCCESS
			!= do_pip3_switch_image_cmd(0x00, PIP3_IMAGE_ID_PRIMARY)) {
		active_dut_state = DUT_STATE_INVALID;
		active_flash_loader = FLASH_LOADER_NONE;
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS != _verify_fw_category(PIP3_FW_CATEGORY_ID_UTILITY_FW)) {
		active_dut_state = DUT_STATE_INVALID;
		active_flash_loader = FLASH_LOADER_NONE;
		return EXIT_FAILURE;
	}

	active_dut_state = DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE;
	active_flash_loader = FLASH_LOADER_NONE;
	return EXIT_SUCCESS;
}

static int _set_dut_state_tp_bl_exec()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	PIP2_Rsp_Payload_Status pip2_status_rsp;
	PIP3_Rsp_Payload_Status pip3_status_rsp;

	if (!is_pip2_api_active()) {
		output(DEBUG,
	"Cannot enter the ROM Bootloader EXEC if the PIP2 API is inactive or\n"
	"\tunavailable. Note that the PIP2 ROM-BL interface is only available via\n"
	"\tTTDL or the I2C-DEV channel (i.e., directly performing I2C read/write\n"
	"\toperations).\n");
		return EXIT_FAILURE;
	}

	fflush(stderr);
	int initial_stderr_fd = dup(STDERR_FILENO);
	int dev_null_fd = open("/dev/null", O_WRONLY);
	dup2(dev_null_fd, STDERR_FILENO);
	close(dev_null_fd);
	rc = do_pip2_status_cmd(0x00, &pip2_status_rsp);
	fflush(stderr);
	dup2(initial_stderr_fd, STDERR_FILENO);
	close(initial_stderr_fd);

	if (rc == EXIT_SUCCESS) {
		active_dut_state = DUT_STATE_TP_BL;
		output(DEBUG, "Already in the %s.\n",
				DUT_STATE_LABELS[DUT_STATE_TP_BL]);
		return rc;
	}

	rc = do_pip3_status_cmd(0x00, &pip3_status_rsp);
	if (rc != EXIT_SUCCESS) {
		output(ERROR, "%s: Neither PIP2 nor PIP3 STATUS cmd worked.\n",
				__func__);
		return rc;
	}

	output(DEBUG,
			"\n"
			"  EXEC:           %s\n"
			"  FW System Mode: %s\n",
			PIP3_EXEC_NAMES[pip3_status_rsp.exec],
			PIP3_APP_SYS_MODE_NAMES[pip3_status_rsp.sys_mode]);

	rc = do_pip3_switch_image_cmd(0x00, PIP3_IMAGE_ID_ROM_BL);
	if (rc != EXIT_SUCCESS) {
		active_dut_state = DUT_STATE_INVALID;
		return rc;
	}

	rc = do_pip2_status_cmd(0x00, &pip2_status_rsp);
	if (rc != EXIT_SUCCESS) {
		active_dut_state = DUT_STATE_INVALID;
		output(ERROR, "%s: Attempted to switch to the % but the PIP2 STATUS cmd"
				" failed.\n",
				__func__, DUT_STATE_LABELS[DUT_STATE_TP_BL]);
		return rc;
	}

	output(DEBUG, "Now in the %s.\n", DUT_STATE_LABELS[DUT_STATE_TP_BL]);
	active_dut_state = DUT_STATE_TP_BL;
	return rc;
}

static int _set_dut_state_tp_fw_exec()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	PIP2_Rsp_Payload_Status pip2_status_rsp;
	PIP3_Rsp_Payload_Status pip3_status_rsp;

	if (active_dut_state == DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE
			|| active_dut_state == DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE) {

		if (active_dut_state == DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE
				&& EXIT_SUCCESS != do_pip3_switch_image_cmd(0x00,
						PIP3_IMAGE_ID_PRIMARY)) {
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		}

		struct timeval now_time;
		long double delta_sec;
		uint duration_remainder_sec;
		gettimeofday(&now_time, 0);
		delta_sec = now_time.tv_sec - aux_mcu_active_start_time.tv_sec
				+ ((now_time.tv_usec - aux_mcu_active_start_time.tv_usec)
						/ USEC_SEC_RATIO);
		output(INFO,
				"Elapsed time since AUX MCU was activated: %.3lf seconds.\n",
				delta_sec);

		duration_remainder_sec = (uint)
				(get_aux_mcu_active_duration_seconds() - delta_sec + 1);
		output(DEBUG,
				"Waiting for %d seconds, at which point the AUX MCU is expected"
				"to be inactive.\n",
				duration_remainder_sec);
		sleep(duration_remainder_sec);
		if (EXIT_SUCCESS
				!= _verify_active_processor(PIP3_PROCESSOR_ID_PRIMARY,
						AUX_MCU_MAX_WAIT_TO_ACTIVATE_SECONDS)) {
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		}
	}

	fflush(stderr);
	int initial_stderr_fd = dup(STDERR_FILENO);
	int dev_null_fd = open("/dev/null", O_WRONLY);
	dup2(dev_null_fd, STDERR_FILENO);
	close(dev_null_fd);
	int cmd_rc = do_pip3_status_cmd(0x00, &pip3_status_rsp);
	fflush(stderr);
	dup2(initial_stderr_fd, STDERR_FILENO);
	close(initial_stderr_fd);

	if (cmd_rc == EXIT_SUCCESS) {
		switch (pip3_status_rsp.exec) {
		case PIP3_EXEC_RAM:
			output(DEBUG, "Already in the %s.\n",
					DUT_STATE_LABELS[DUT_STATE_TP_FW_SYS_MODE_ANY]);

			PIP3_Rsp_Payload_Version pip3_version_rsp;
			if (EXIT_SUCCESS != do_pip3_version_cmd(0x00, &pip3_version_rsp)) {
				active_dut_state = DUT_STATE_INVALID;
				return EXIT_FAILURE;
			}

			if (PIP3_FW_CATEGORY_ID_PROGRAMMER_FW == pip3_version_rsp.fw_category_id
				|| PIP3_FW_CATEGORY_ID_BOOTLOADER_FW == pip3_version_rsp.fw_category_id)
			{
				active_flash_loader = FLASH_LOADER_TP_PROGRAMMER_IMAGE;
				active_dut_state = DUT_STATE_TP_FW_PROGRAMMER_IMAGE;
			} else {
				active_flash_loader = FLASH_LOADER_NONE;
				active_dut_state = _get_dut_state_from_fw_sys_mode(pip3_status_rsp.sys_mode);
			}

			return EXIT_SUCCESS;
		case PIP3_EXEC_ROM:
			output(ERROR,
					"%s: Stuck in the PIP3 ROM-BL, which is not yet supported "
					"by PtTools.\n",
					__func__);
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		default:
			output(ERROR, "%s: Unrecognized EXEC.\n", __func__);
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		}
	}

	if (!is_pip2_api_active()) {
		output(ERROR,
				"%s: The DUT is stuck in the ROM Bootloader. Cannot exit\n"
				"(or enter) without access to the PIP2 ROM-BL interface,\n"
				"which is only available via TTDL or the I2C-DEV channel\n"
				"(i.e., directly performing I2C read/write operations).\n",
				__func__);
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS != do_pip2_status_cmd(0x00, &pip2_status_rsp)) {
		output(ERROR,
				"%s: Neither PIP3 nor PIP2 STATUS cmd worked. The DUT is in an "
				"unknown state. This could imply that the PIP2 ROM-BL image is "
				"corrupt/invalid.\n",
				__func__);
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	output(DEBUG,
			"\n"
			"  EXEC:           %s\n"
			"  FW System Mode: %s\n",
			PIP2_EXEC_NAMES[pip2_status_rsp.exec],
			PIP2_APP_SYS_MODE_NAMES[pip2_status_rsp.sys_mode]);

	if (EXIT_SUCCESS != do_pip2_reset_cmd(0x00)) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS != do_pip3_status_cmd(0x00, &pip3_status_rsp)) {
		output(ERROR,
				"%s: Executed PIP2 RESET to try to exit the ROM Bootloader but "
				"still unable to communicate with the .\n", __func__);
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	if (PIP3_EXEC_RAM != pip3_status_rsp.exec) {
		output(ERROR,
				"%s: Executed PIP2 RESET to try to exit the ROM Bootloader but "
				"failed to enter the RAM Application EXEC.\n"
				"  EXEC:           %s\n"
				"  FW System Mode: %s\n",
				__func__,
				PIP3_EXEC_NAMES[pip3_status_rsp.exec],
				PIP3_APP_SYS_MODE_NAMES[pip3_status_rsp.sys_mode]);
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	output(DEBUG, "Now in the %s.\n",
			DUT_STATE_LABELS[DUT_STATE_TP_FW_SYS_MODE_ANY]);
	active_dut_state = DUT_STATE_TP_FW_SYS_MODE_ANY;
	return EXIT_SUCCESS;
}

static int _set_dut_state_tp_fw_scanning()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	PIP3_Rsp_Payload_ResumeScanning resume_scan_rsp;
	PIP3_Rsp_Payload_Status status_rsp;
	int wait_time_remaining = BOOT_2_SCANNING_MAX_WAIT_MS;

	if (EXIT_SUCCESS != _set_dut_state_tp_fw_exec()) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	switch (active_dut_state) {
	case DUT_STATE_TP_FW_SCANNING:
		return EXIT_SUCCESS;

	case DUT_STATE_TP_FW_PROGRAMMER_IMAGE:
		if (EXIT_SUCCESS
				!= do_pip3_switch_image_cmd(0x00, PIP3_IMAGE_ID_PRIMARY)) {
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		}

		if (EXIT_SUCCESS != _verify_fw_category(PIP3_FW_CATEGORY_ID_TOUCH_FW)) {
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		}

		active_flash_loader = FLASH_LOADER_NONE;

		break;
	default:
		;
	}

	rc = do_pip3_status_cmd(0x00, &status_rsp);
	if (rc != EXIT_SUCCESS) {
		return rc;
	}

	output(DEBUG,
			"\n"
			"  EXEC:           %s\n"
			"  FW System Mode: %s\n",
			PIP3_EXEC_NAMES[status_rsp.exec],
			PIP3_APP_SYS_MODE_NAMES[status_rsp.sys_mode]);

	if (status_rsp.exec != PIP3_EXEC_RAM) {
		output(ERROR,
				"%s: The DUT is stuck in the bootloader. Cannot exit (or enter)"
				"the PIP2 Bootloader when using drivers/channels other than "
				"TTDL.\n",
				__func__);
		return EXIT_FAILURE;
	}

	switch (status_rsp.sys_mode) {
	case PIP3_APP_SYS_MODE_BOOT:
		while (wait_time_remaining > 0
				&& status_rsp.sys_mode == PIP3_APP_SYS_MODE_BOOT) {
			sleep_ms(BOOT_2_SCANNING_POLLING_INTERVAL_MS);
			wait_time_remaining -= BOOT_2_SCANNING_POLLING_INTERVAL_MS;
			output(DEBUG, "wait_time_remaining: %d.\n",  wait_time_remaining);
			if ( ((BOOT_2_SCANNING_MAX_WAIT_MS - wait_time_remaining) %
					BOOT_2_SCANNING_INFO_MESSAGE_INTERVAL_MS) == 0) {
				output(INFO, "Waiting for FW to exit boot mode.\n");
			}
			rc = do_pip3_status_cmd(0x00, &status_rsp);
			if (rc != EXIT_SUCCESS) {
				return rc;
			}
		}
		if (wait_time_remaining <= 0) {
			output(ERROR, "Timeout waiting for FW to exit boot mode.\n");
			rc = EXIT_FAILURE;
		}
		break;
	case PIP3_APP_SYS_MODE_SCANNING:
		output(DEBUG, "Already in %s.\n",
				PIP3_APP_SYS_MODE_NAMES[PIP3_APP_SYS_MODE_SCANNING]);
		rc = EXIT_SUCCESS;
		break;
	case PIP3_APP_SYS_MODE_DEEP_SLEEP:
		output(ERROR, "%s: Switching from %s to %s is not implemented yet.\n",
				__func__,
				PIP3_APP_SYS_MODE_NAMES[PIP3_APP_SYS_MODE_DEEP_SLEEP],
				PIP3_APP_SYS_MODE_NAMES[PIP3_APP_SYS_MODE_SCANNING]);
		rc = EXIT_FAILURE;
		break;
	case PIP3_APP_SYS_MODE_TEST_CONFIG:
		rc = do_pip3_resume_scanning_cmd(0x00, &resume_scan_rsp);
		if (rc != EXIT_SUCCESS) {
			break;
		}

		if (do_pip3_status_cmd(0x00, &status_rsp) != EXIT_SUCCESS
				|| status_rsp.sys_mode != PIP3_APP_SYS_MODE_SCANNING) {
			rc = EXIT_FAILURE;
			break;
		}
		break;
	case PIP3_APP_SYS_MODE_DEEP_STANDBY:
		output(ERROR,
				"%s: The only way to exit %s is to toggle the external reset "
				"but this action is not currently supported with "
				"drivers/channels other than TTDL.\n",
				__func__);
		rc = EXIT_FAILURE;
		break;
	default:
		output(ERROR, "%s: Unexpected FW System Mode: 0x%02X.\n", __func__,
				status_rsp.sys_mode);
		rc = EXIT_FAILURE;
	}

	active_flash_loader = FLASH_LOADER_NONE;
	active_dut_state = (rc == EXIT_SUCCESS) ? DUT_STATE_TP_FW_SCANNING : DUT_STATE_INVALID;

	return rc;
}

static int _set_dut_state_tp_fw_test()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	int rc = EXIT_FAILURE;
	PIP3_Rsp_Payload_SuspendScanning suspend_scan_rsp;
	PIP3_Rsp_Payload_Status status_rsp;
	int wait_time_remaining = BOOT_2_SCANNING_MAX_WAIT_MS;

	if (EXIT_SUCCESS != _set_dut_state_tp_fw_exec()) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	switch (active_dut_state) {
	case DUT_STATE_TP_FW_TEST:
		return EXIT_SUCCESS;

	case DUT_STATE_TP_FW_PROGRAMMER_IMAGE:
		if (EXIT_SUCCESS
				!= do_pip3_switch_image_cmd(0x00, PIP3_IMAGE_ID_PRIMARY)) {
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		}

		if (EXIT_SUCCESS != _verify_fw_category(PIP3_FW_CATEGORY_ID_TOUCH_FW)) {
			active_dut_state = DUT_STATE_INVALID;
			return EXIT_FAILURE;
		}

		active_flash_loader = FLASH_LOADER_NONE;

		break;
	default:
		;
	}

	rc = do_pip3_status_cmd(0x00, &status_rsp);
	if (rc != EXIT_SUCCESS) {
		return rc;
	}

	output(DEBUG,
			"\n"
			"  EXEC:           %s\n"
			"  FW System Mode: %s\n",
			PIP3_EXEC_NAMES[status_rsp.exec],
			PIP3_APP_SYS_MODE_NAMES[status_rsp.sys_mode]);

	if (status_rsp.exec != PIP3_EXEC_RAM) {
		output(ERROR,
				"%s: The DUT is stuck in the bootloader. Cannot exit (or enter)"
				"the PIP2 Bootloader when using drivers/channels other than "
				"TTDL.\n",
				__func__);
		return EXIT_FAILURE;
	}

	switch (status_rsp.sys_mode) {
	case PIP3_APP_SYS_MODE_BOOT:
		while (wait_time_remaining > 0
				&& status_rsp.sys_mode == PIP3_APP_SYS_MODE_BOOT) {
			sleep_ms(BOOT_2_SCANNING_POLLING_INTERVAL_MS);
			wait_time_remaining -= BOOT_2_SCANNING_POLLING_INTERVAL_MS;
			output(DEBUG, "wait_time_remaining: %d.\n",  wait_time_remaining);
			if ( ((BOOT_2_SCANNING_MAX_WAIT_MS - wait_time_remaining) %
					BOOT_2_SCANNING_INFO_MESSAGE_INTERVAL_MS) == 0) {
				output(INFO, "Waiting for FW to exit boot mode.\n");
			}
			rc = do_pip3_status_cmd(0x00, &status_rsp);
			if (rc != EXIT_SUCCESS) {
				return rc;
			}
		}
		if (wait_time_remaining <= 0) {
			output(ERROR, "Timeout waiting for FW to exit boot mode.\n");
			rc = EXIT_FAILURE;
			break;
		}
		rc = do_pip3_suspend_scanning_cmd(0x00, &suspend_scan_rsp);
		if (rc != EXIT_SUCCESS) {
			break;
		}

		if (do_pip3_status_cmd(0x00, &status_rsp) != EXIT_SUCCESS
				|| status_rsp.sys_mode != PIP3_APP_SYS_MODE_TEST_CONFIG) {
			rc = EXIT_FAILURE;
			break;
		}
		break;
	case PIP3_APP_SYS_MODE_TEST_CONFIG:
		output(DEBUG, "Already in %s.\n",
				PIP3_APP_SYS_MODE_NAMES[PIP3_APP_SYS_MODE_TEST_CONFIG]);
		rc = EXIT_SUCCESS;
		break;
	case PIP3_APP_SYS_MODE_DEEP_SLEEP:
		output(ERROR, "%s: Switching from %s to %s is not implemented yet.\n",
				__func__,
				PIP3_APP_SYS_MODE_NAMES[PIP3_APP_SYS_MODE_DEEP_SLEEP],
				PIP3_APP_SYS_MODE_NAMES[PIP3_APP_SYS_MODE_SCANNING]);
		rc = EXIT_FAILURE;
		break;
	case PIP3_APP_SYS_MODE_SCANNING:
		rc = do_pip3_suspend_scanning_cmd(0x00, &suspend_scan_rsp);
		break;
	case PIP3_APP_SYS_MODE_DEEP_STANDBY:
		output(ERROR,
				"%s: The only way to exit %s is to toggle the external reset "
				"but this action is not currently supported with "
				"drivers/channels other than TTDL.\n",
				__func__);
		rc = EXIT_FAILURE;
		break;
	default:
		output(ERROR, "%s: Unexpected FW System Mode: 0x%02X.\n", __func__,
				status_rsp.sys_mode);
		rc = EXIT_FAILURE;
	}

	active_flash_loader = FLASH_LOADER_NONE;
	active_dut_state = (rc == EXIT_SUCCESS) ? DUT_STATE_TP_FW_TEST : DUT_STATE_INVALID;

	return rc;
}

static int _set_dut_state_tp_programmer_img()
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (!is_pip3_api_active()) {
		output(DEBUG,
				"Cannot enter the Secondary Loader if the PIP3 API is\n"
				"inactive/unavailable.\n");
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}
	PIP3_Rsp_Payload_Version version_rsp;

	if (EXIT_SUCCESS != do_pip3_version_cmd(0x00, &version_rsp)) {
		return EXIT_FAILURE;
	}

	if (version_rsp.fw_category_id == PIP3_FW_CATEGORY_ID_PROGRAMMER_FW
		|| version_rsp.fw_category_id == PIP3_FW_CATEGORY_ID_BOOTLOADER_FW) {
		active_dut_state = DUT_STATE_TP_FW_PROGRAMMER_IMAGE;
		output(DEBUG, "%s: Already running from %s.\n",
				__func__,
				PIP3_FW_CATEGORY_NAMES[version_rsp.fw_category_id]);
		return EXIT_ALREADY_IN_REQUESTED_STATE;
	}

	if (EXIT_SUCCESS
			!= do_pip3_switch_image_cmd(0x00, PIP3_IMAGE_ID_SECONDARY)) {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS
		== _verify_fw_category(PIP3_FW_CATEGORY_ID_PROGRAMMER_FW)) {
		active_dut_state = DUT_STATE_TP_FW_PROGRAMMER_IMAGE;
		output(DEBUG, "Entered the Programmer Image.\n");
		return EXIT_SUCCESS;
	} else if (EXIT_SUCCESS
		== _verify_fw_category(PIP3_FW_CATEGORY_ID_BOOTLOADER_FW)) {
		active_dut_state = DUT_STATE_TP_FW_PROGRAMMER_IMAGE;
		output(DEBUG, "Entered the Bootloader FW.\n");
		return EXIT_SUCCESS;
	} else {
		active_dut_state = DUT_STATE_INVALID;
		return EXIT_FAILURE;
	}
}

#define VERIFY_ACTIVE_PROCESSOR_INTERVAL_BETWEEN_MSGS_SECS 3
#define INTERVAL_BETWEEN_PIP3_STATUS_CMDS_MSECS 200

static int _verify_active_processor(PIP3_Processor_ID expected_processor,
		long double timeout_seconds)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	struct timeval start_time;
	PIP3_Processor_ID active_processor;
	struct timeval previous_msg_time;
	bool first_msg = true;

	gettimeofday(&start_time, 0);
	gettimeofday(&previous_msg_time, 0);
	do {
		PIP3_Rsp_Payload_Status status_rsp;

		if (EXIT_SUCCESS != do_pip3_status_cmd(0x00, &status_rsp)) {
			return EXIT_FAILURE;
		}

		active_processor = status_rsp.active_processor;
		if (expected_processor == active_processor) {
			return EXIT_SUCCESS;
		} else if (time_limit_reached(&start_time, timeout_seconds)) {
			output(DEBUG,
"%s: Response to the PIP3 STATUS command indicates that the currently active\n"
"\t  processor is unexpected (expected %s, got %s).\n",
					__func__,
					PIP3_PROCESSOR_NAMES[expected_processor],
					PIP3_PROCESSOR_NAMES[active_processor]);

			return EXIT_FAILURE;
		}

		struct timeval now_time;
		long double delta;
		gettimeofday(&now_time, 0);
		delta = now_time.tv_sec - previous_msg_time.tv_sec
				+ ((now_time.tv_usec - previous_msg_time.tv_usec)
						/ USEC_SEC_RATIO);
		if (first_msg ||
				delta >= VERIFY_ACTIVE_PROCESSOR_INTERVAL_BETWEEN_MSGS_SECS) {
			output(INFO, "Waiting for %s to become the active processor.\n",
					PIP3_PROCESSOR_NAMES[expected_processor]);
			gettimeofday(&previous_msg_time, 0);
			first_msg = false;
		}

		sleep_ms(INTERVAL_BETWEEN_PIP3_STATUS_CMDS_MSECS);
	} while (expected_processor != active_processor);

	return EXIT_FAILURE;
}

static int _verify_fw_category(PIP3_FW_Category_ID expected_fw_category_id)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	PIP3_Rsp_Payload_Version version_rsp;

	if (EXIT_SUCCESS != do_pip3_version_cmd(0x00, &version_rsp)) {
		return EXIT_FAILURE;
	}

	if (version_rsp.fw_category_id != expected_fw_category_id) {
		output(DEBUG,
"%s: Response to the PIP3 VERSION command indicates that the active firmware\n"
"\t  is not the expected category (expected %s, got %s).\n",
				__func__,
				PIP3_FW_CATEGORY_NAMES[expected_fw_category_id],
				PIP3_FW_CATEGORY_NAMES[version_rsp.fw_category_id]);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
