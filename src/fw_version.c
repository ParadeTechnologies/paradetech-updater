/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "fw_version.h"

static int get_fw_version_from_ready_dut(FW_Version* version)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	PIP3_Rsp_Payload_Status status_rsp;
	PIP3_Rsp_Payload_Version version_rsp;
	PIP3_Rsp_Payload_GetSysinfo rsp;

	if (EXIT_SUCCESS != do_pip3_status_cmd(0x00, &status_rsp)) {
		return EXIT_FAILURE;
		/* NOTREACHED */
	}

	if (EXIT_SUCCESS != do_pip3_version_cmd(0x00, &version_rsp)) {
		return EXIT_FAILURE;
		/* NOTREACHED */
	}

	if (status_rsp.active_processor != PIP3_PROCESSOR_ID_PRIMARY ||
		status_rsp.exec != PIP3_EXEC_RAM ||
		status_rsp.sys_mode != PIP3_APP_SYS_MODE_SCANNING ||
		version_rsp.fw_category_id != PIP3_FW_CATEGORY_ID_TOUCH_FW) {
		output(WARNING,
			"%s: DUT state is not ready:\n"
			"       PROC: %s\n"
			"       EXEC: %s\n"
			"       MODE: %s\n"
			"       CATA: %s\n",
			__func__,
			PIP3_EXEC_NAMES[status_rsp.exec],
			PIP3_PROCESSOR_NAMES[status_rsp.active_processor],
			PIP3_APP_SYS_MODE_NAMES[status_rsp.sys_mode],
			PIP3_FW_CATEGORY_NAMES[version_rsp.fw_category_id]);
		return EXIT_FAILURE;
	}

	if (EXIT_SUCCESS != do_pip3_get_sysinfo_cmd(0x00, &rsp)) {
		return EXIT_FAILURE;
		/* NOTREACHED */
	}

	version->major       = rsp.fw_major_version;
	version->minor       = rsp.fw_minor_version;
	version->rev_control = (
			   rsp.fw_rev_control_num[0]
			+ (rsp.fw_rev_control_num[1] << 8)
			+ (rsp.fw_rev_control_num[2] << 8 * 2)
			+ (rsp.fw_rev_control_num[3] << 8 * 3));
	version->config_ver  = (
			rsp.fw_config_version[0] + (rsp.fw_config_version[1] << 8));

    return EXIT_SUCCESS;
}

#define TOUCH_MCU_READY_MAX_WAIT_MS (5000)
#define T_READY_MAX_WAIT_MS (200)
static int do_wait_touch_mcu_active()
{
	output(DEBUG, "%s: Starting.\n", __func__);
	PIP3_Rsp_Payload_Status status_rsp;
	bool mcu_ready = false;
	bool need_fw_loading_delay = false;
	int wait_time_remaining = TOUCH_MCU_READY_MAX_WAIT_MS;

	/* Wait touch mcu ready */
	while (wait_time_remaining > 0) {
		if (EXIT_SUCCESS == do_pip3_status_cmd(0x00, &status_rsp)) {
			if (status_rsp.active_processor == PIP3_PROCESSOR_ID_PRIMARY) {
				output(DEBUG, "Touch MCU is ready.\n");
				mcu_ready = true;
				break;
			}

		}
		sleep_ms(100);
		wait_time_remaining -= 100;
		need_fw_loading_delay = true;
	}

	if (!mcu_ready) {
		output(ERROR,
			"%s: Timeout waiting for the Touch MCU to become ready.\n"
			"  EXEC: %s\n  PROC: %s\n",
			__func__,
			PIP3_EXEC_NAMES[status_rsp.exec],
			PIP3_PROCESSOR_NAMES[status_rsp.active_processor]);
		return EXIT_FAILURE;
	}

	if (need_fw_loading_delay) {
		output(DEBUG, "Touch MCU is ready, but fw is loading.\n");
		sleep_ms(T_READY_MAX_WAIT_MS);
	}

	return EXIT_SUCCESS;
}

int get_fw_version_from_flash(FW_Version* version)
{
	output(DEBUG, "%s: Starting.\n", __func__);
	PIP3_Rsp_Payload_GetSysinfo rsp;
	bool dut_state_stuck = false;

	if (version == NULL) {
		output(ERROR, "%s: NULL argument given.\n", __func__);
		return EXIT_FAILURE;
	}

	/*
	 * Temporarily redirect stderr logs to /dev/null so the user does not see
	 * various ERROR messages for issues that suggest the firmware simply needs
	 * to be updated (e.g., PIP3 STATUS command does not work, or if the DUT is
	 * stuck in the ROM Bootloader). So instead, we just show the active
	 * firmware version as "0.0.0.0" to ensure/force a firmware update.
	 */
	fflush(stderr);
	int initial_stderr_fd = dup(STDERR_FILENO);
	int dev_null_fd = open("/dev/null", O_WRONLY);
	dup2(dev_null_fd, STDERR_FILENO);
	close(dev_null_fd);

	if (EXIT_SUCCESS == get_fw_version_from_ready_dut(version)) {
		return EXIT_SUCCESS;
		/* NOTREACHED */
	}
	
	output(INFO,
		"%s: Get version failed. Trying to get the DUT into "
		"the desired state.\n", __func__);

	if (EXIT_SUCCESS != do_wait_touch_mcu_active()) {
		output(INFO, "%s: Failed to wait touch mcu ready\n",
			__func__);
		return EXIT_FAILURE;
		/* NOTREACHED */
	}

	if (EXIT_SUCCESS != set_dut_state(DUT_STATE_TP_FW_SCANNING)) {
		dut_state_stuck = true;
		output(DEBUG,
			"Unable to get DUT into desired state (%s). This suggests that a "
			"firmware update is required.\n",
			DUT_STATE_LABELS[DUT_STATE_TP_FW_SCANNING]);
		version->major = 0;
		version->minor = 0;
		version->rev_control = 0;
		version->config_ver = 0;
	}

	fflush(stderr);
	dup2(initial_stderr_fd, STDERR_FILENO);
	close(initial_stderr_fd);

	if (dut_state_stuck) {
		return EXIT_SUCCESS;
		/* NOTREACHED */
	}

	if (EXIT_SUCCESS != do_pip3_get_sysinfo_cmd(0x00, &rsp)) {
		return EXIT_FAILURE;
		/* NOTREACHED */
	}

	version->major       = rsp.fw_major_version;
	version->minor       = rsp.fw_minor_version;
	version->rev_control = (
			   rsp.fw_rev_control_num[0]
			+ (rsp.fw_rev_control_num[1] << 8)
			+ (rsp.fw_rev_control_num[2] << 8 * 2)
			+ (rsp.fw_rev_control_num[3] << 8 * 3));
	version->config_ver  = (
			rsp.fw_config_version[0] + (rsp.fw_config_version[1] << 8));

    return EXIT_SUCCESS;
}

int get_fw_version_from_bin_header(const FW_Bin_Header* bin_header,
	FW_Version* version)
{
	output(DEBUG, "%s: Starting.\n", __func__);

	if (bin_header == NULL || version == NULL) {
		output(ERROR, "%s: NULL argument given.\n", __func__);
		return EXIT_FAILURE;
	}

	version->major       = bin_header->fw_major_version;
	version->minor       = bin_header->fw_minor_version;
	version->rev_control = (
		  (bin_header->fw_rev_control[0] << 8 * 3)
		+ (bin_header->fw_rev_control[1] << 8 * 2)
		+ (bin_header->fw_rev_control[2] << 8)
		+  bin_header->fw_rev_control[3]);
	version->silicon_id =
		(bin_header->silicon_id[1] << 8) +  bin_header->silicon_id[0];
	version->config_ver = (
		(bin_header->config_version[0] << 8) +  bin_header->config_version[1]);

	return EXIT_SUCCESS;
}
