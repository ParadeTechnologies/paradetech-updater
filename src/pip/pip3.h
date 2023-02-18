/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_PIP3_H_
#define PTLIB_PIP3_H_

#include "../base64.h"
#include "../channel.h"
#include "../crc16_ccitt.h"
#include "../hid/hidraw.h"
#include "pip3_cmd_id.h"
#include "pip3_self_test_id.h"
#include "pip3_status_code.h"

#define HID_INPUT_PIP3_RSP_PAYLOAD_START_BYTE_INDEX  2

typedef enum {
	PIP3_EXEC_ROM = 0x00,
	PIP3_EXEC_RAM = 0x01,
	NUM_OF_PIP3_EXECS = 2
} PIP3_Exec;

extern char* PIP3_EXEC_NAMES[NUM_OF_PIP3_EXECS];

typedef enum {
	PIP3_APP_SYS_MODE_BOOT            = 0x00,
	PIP3_APP_SYS_MODE_SCANNING        = 0x01,
	PIP3_APP_SYS_MODE_DEEP_SLEEP      = 0x02,
	PIP3_APP_SYS_MODE_TEST_CONFIG     = 0x03,
	PIP3_APP_SYS_MODE_DEEP_STANDBY    = 0x04,
	PIP3_APP_SYS_MODE_SECONDARY_IMAGE = 0x05,
	NUM_OF_PIP3_APP_SYS_MODES = 6
} PIP3_App_Sys_Mode;

extern char* PIP3_APP_SYS_MODE_NAMES[NUM_OF_PIP3_APP_SYS_MODES];

typedef enum {
	PIP3_IMAGE_ID_PRIMARY   = 0x00,
	PIP3_IMAGE_ID_SECONDARY = 0x01,
	PIP3_IMAGE_ID_ROM_BL    = 0x02,

	NUM_OF_PIP3_IMAGE_IDS  = 3
} PIP3_Image_ID;

extern char* PIP3_IMAGE_NAMES[NUM_OF_PIP3_IMAGE_IDS];

typedef enum {
	PIP3_IOCTL_CODE_ERASE_FILE         = 0x00,
	PIP3_IOCTL_CODE_SEEK_FILE_POINTERS = 0x01,
	PIP3_IOCTL_CODE_AES_CONTROL        = 0x02,
	PIP3_IOCTL_CODE_FILE_STATS         = 0x03,
	PIP3_IOCTL_CODE_FILE_CRC           = 0x04,

	NUM_OF_PIP3_IOCTL_CODES = 5
} PIP3_IOCTL_Code;

extern char* PIP3_IOCTL_CODE_LABELS[NUM_OF_PIP3_IOCTL_CODES];

typedef struct {
	uint8_t report_id;                  
	uint8_t payload_len_lsb;            
	uint8_t payload_len_msb;            
	struct {                            
		uint8_t seq                : 3;
		uint8_t tag                : 1;
		uint8_t more_data          : 1;
		uint8_t reserved_section_1 : 3;
	} __attribute__((packed));
	struct {                            
		uint8_t cmd_id : 7;
		uint8_t resp   : 1;
	} __attribute__((packed));
} __attribute__((packed)) PIP3_Cmd_Header;

typedef struct {
	uint8_t crc_msb;
	uint8_t crc_lsb;
} __attribute__((packed)) PIP3_Cmd_Footer;

typedef struct {
	uint8_t payload_len_lsb;            
	uint8_t payload_len_msb;            
	struct {                            
		uint8_t seq                : 3;
		uint8_t tag                : 1;
		uint8_t more_data          : 1;
		uint8_t reserved_section_1 : 3;
	} __attribute__((packed));
	struct {                            
		uint8_t cmd_id : 7;
		uint8_t resp   : 1;
	} __attribute__((packed));
	uint8_t status_code;                
} __attribute__((packed)) PIP3_Rsp_Header;

typedef struct {
	uint8_t crc_msb;
	uint8_t crc_lsb;
} __attribute__((packed)) PIP3_Rsp_Footer;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t data_id_mask;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_InitializeBaselines;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_InitializeBaselines;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t file_handle;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_FileClose;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_FileClose;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t file_handle;
	uint8_t ioctl_code;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_FileIOCTL_EraseFile;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_FileIOCTL_EraseFile;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t file_num;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_FileOpen;

typedef struct {
	PIP3_Rsp_Header header;
	uint8_t file_handle;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_FileOpen;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t file_handle;
	uint8_t read_len_lsb;
	uint8_t read_len_msb;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_FileRead;

typedef struct {
	PIP3_Rsp_Header header;
	uint8_t* data;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_FileRead;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t file_handle;
	uint8_t* data;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_FileWrite;

#define PIP3_FILE_WRITE_CMD_WITHOUT_DATA_LEN \
	sizeof(PIP3_Cmd_Payload_FileWrite) - sizeof(uint8_t*)

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_FileWrite;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t read_offset_lsb;
	uint8_t read_offset_msb;
	uint8_t read_len_lsb;
	uint8_t read_len_msb;
	uint8_t self_test_id;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_GetSelfTestResults;

typedef struct {
	PIP3_Rsp_Header header;             
	uint8_t self_test_id;               
	struct {                            
		uint8_t data_format_id : 4;
		uint8_t data_unit_id   : 4;
	} __attribute__((packed));
	uint8_t arl_lsb;                    
	uint8_t arl_msb;                    
	uint8_t* data;                      
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_GetSelfTestResults;

typedef struct {
	PIP3_Cmd_Header header;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_GetSysinfo;

typedef struct { 

	PIP3_Rsp_Header header;

	uint8_t pip_major_version;
	uint8_t pip_minor_version;
	uint8_t touch_fw_product_id[2];
	uint8_t fw_major_version;
	uint8_t fw_minor_version;
	uint8_t fw_rev_control_num[4];
	uint8_t fw_config_version[2];
	uint8_t bl_major_version;
	uint8_t bl_minor_version;
	uint8_t family_id;
	uint8_t revision_id;
	uint8_t silicon_id[2];
	uint8_t parade_mfg_id_0;
	uint8_t parade_mfg_id_1;
	uint8_t parade_mfg_id_2;
	uint8_t parade_mfg_id_3;
	uint8_t parade_mfg_id_4;
	uint8_t parade_mfg_id_5;
	uint8_t parade_mfg_id_6;
	uint8_t parade_mfg_id_7;
	uint8_t post_results_code[2];

	uint8_t num_of_electrodes_x_axis;
	uint8_t num_of_electrodes_y_axis;
	uint8_t panel_x_axis_len[2];
	uint8_t panel_y_axis_len[2];
	uint8_t panel_x_res[2];
	uint8_t panel_y_res[2];
	uint8_t panel_pressure_res[2];
	struct {
		uint8_t x_org              : 1;
		uint8_t x_is_tx            : 1;
		uint8_t reserved_section_1 : 6;
	} __attribute__((packed));
	struct {
		uint8_t y_org              : 1;
		uint8_t reserved_section_2 : 7;
	} __attribute__((packed));
	struct {
		uint8_t panel_id           : 4;
		uint8_t reserved_section_3 : 4;
	} __attribute__((packed));
	struct {
		uint8_t btn_1_exist : 1;
		uint8_t btn_2_exist : 1;
		uint8_t btn_3_exist : 1;
		uint8_t btn_4_exist : 1;
		uint8_t btn_5_exist : 1;
		uint8_t btn_6_exist : 1;
		uint8_t btn_7_exist : 1;
		uint8_t btn_8_exist : 1;
	} __attribute__((packed));
	struct {
		uint8_t mc                 : 1;
		uint8_t self               : 1;
		uint8_t bal                : 1;
		uint8_t reserved_section_4 : 5;
	} __attribute__((packed));
	uint8_t max_num_of_touch_records;
	uint8_t num_of_force_electrodes_x_axis;
	uint8_t num_of_force_electrodes_y_axis;
	struct {
		uint8_t force_dim          : 1;
		uint8_t reserved_section_5 : 7;
	} __attribute__((packed));

	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_GetSysinfo;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t load_offset_lsb;
	uint8_t load_offset_msb;
	uint8_t load_len_lsb;
	uint8_t load_len_msb;
	uint8_t self_test_id;
	uint8_t* param_data;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_LoadSelfTestParam;

#define PIP3_LOAD_SELF_TEST_PARAM_CMD_WITHOUT_PARAM_DATA_LEN \
	sizeof(PIP3_Cmd_Payload_LoadSelfTestParam) - sizeof(uint8_t*)
#define PIP3_LOAD_SELF_TEST_PARAM_CMD_PARAM_DATA_START_BYTE_INDEX 10
#define PIP3_LOAD_SELF_TEST_PARAM_CMD_PARAM_DATA_MAX_LEN \
	0xFFFF - PIP3_LOAD_SELF_TEST_PARAM_CMD_WITHOUT_PARAM_DATA_LEN

typedef struct {
	PIP3_Rsp_Header header;
	uint8_t self_test_id;
	uint8_t all_lsb;
	uint8_t all_msb;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_LoadSelfTestParam;

typedef struct {
	PIP3_Cmd_Header header;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_ResumeScanning;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_ResumeScanning;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t self_test_id;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_RunSelfTest;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_RunSelfTest;

typedef struct {
	PIP3_Cmd_Header header;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_StartTrackingHeatmap;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_StartTrackingHeatmap;

#define PIP3_RSP_HEATMAP_PAYLOAD_MAX_LEN 208

typedef struct {
	PIP3_Cmd_Header header;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_Status;

typedef struct {
	PIP3_Rsp_Header header;             
	struct {                            
		uint8_t exec               : 1;
		uint8_t reserved_section_2 : 7;
	} __attribute__((packed));
	uint8_t sys_mode;                   
	struct {                            
		uint8_t protocol_mode      : 3;
		uint8_t reserved_section_3 : 5;
	} __attribute__((packed));
	uint8_t reserved_section_4;         
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_Status;

typedef struct {
	PIP3_Cmd_Header header;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_StopAsyncDebugData;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_StopAsyncDebugData;

typedef struct {
	PIP3_Cmd_Header header;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_SuspendScanning;

typedef struct {
	PIP3_Rsp_Header header;
	PIP3_Rsp_Footer footer;
} __attribute__((packed)) PIP3_Rsp_Payload_SuspendScanning;

typedef struct {
	PIP3_Cmd_Header header;
	uint8_t image_id;
	PIP3_Cmd_Footer footer;
} __attribute__((packed)) PIP3_Cmd_Payload_SwitchImage;

int (*send_report_via_channel)(const ReportData* report);

Poll_Status (*get_report_via_channel)(ReportData* report, bool apply_timeout,
		long double timeout_val);

extern int do_pip3_command(ReportData* cmd, ReportData* rsp);
extern int do_pip3_initialize_baselines_cmd(uint8_t seq_num,
		uint8_t data_id_mask, PIP3_Rsp_Payload_InitializeBaselines* rsp);
extern int do_pip3_file_close_cmd(uint8_t seq_num, uint8_t file_handle,
		PIP3_Rsp_Payload_FileClose* rsp);
extern int do_pip3_file_ioctl_erase_file_cmd(uint8_t seq_num,
		uint8_t file_handle, PIP3_Rsp_Payload_FileIOCTL_EraseFile* rsp);
extern int do_pip3_file_open_cmd(uint8_t seq_num, uint8_t file_num,
		PIP3_Rsp_Payload_FileOpen* rsp);
extern int do_pip3_file_read_cmd(uint8_t seq_num, uint8_t file_handle,
		uint16_t read_len, PIP3_Rsp_Payload_FileRead* rsp, size_t max_rsp_size);
extern int do_pip3_file_write_cmd(uint8_t seq_num, uint8_t file_handle,
		ByteData* data);
extern int do_pip3_get_self_test_results_cmd(uint8_t seq_num,
		uint8_t self_test_id, PIP3_Rsp_Payload_GetSelfTestResults* rsp,
		size_t max_rsp_size);
extern int do_pip3_get_sysinfo_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_GetSysinfo* rsp);
extern int do_pip3_load_self_test_param_cmd(uint8_t seq_num,
		uint8_t self_test_id, ByteData* param_data);
extern int do_pip3_resume_scanning_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_ResumeScanning* rsp);
extern int do_pip3_run_self_test_cmd(uint8_t seq_num, uint8_t self_test_id,
		PIP3_Rsp_Payload_RunSelfTest* rsp);
extern int do_pip3_start_tracking_heatmap_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_StartTrackingHeatmap* rsp);
extern int do_pip3_status_cmd(uint8_t seq_num, PIP3_Rsp_Payload_Status* rsp);
extern int do_pip3_stop_async_debug_data_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_StopAsyncDebugData* rsp);
extern int do_pip3_suspend_scanning_cmd(uint8_t seq_num,
		PIP3_Rsp_Payload_SuspendScanning* rsp);
extern int do_pip3_switch_image_cmd(uint8_t seq_num, PIP3_Image_ID image_id);
extern Poll_Status get_pip3_unsolicited_async_rsp(ReportData* rsp,
		bool apply_timeout, long double timeout_val);
extern bool is_pip3_api_active();
extern int setup_pip3_api(ChannelType channel_type, HID_Report_ID report_id);
extern int teardown_pip3_api();

#endif 