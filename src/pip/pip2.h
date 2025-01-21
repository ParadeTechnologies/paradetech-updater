/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_PIP2_H_
#define PTLIB_PIP2_H_

#include "../base64.h"
#include "../channel/channel.h"
#include "../crc16_ccitt.h"
#include "../file/ptlib_file.h"
#include "../I2C/i2cbusses.h"
#include "../report_data.h"
#include "../sleep/ptlib_sleep.h"
#include "pip2_cmd_id.h"
#include "pip2_status_code.h"

typedef enum {
	PIP2_EXEC_ROM = 0x00,
	PIP2_EXEC_RAM = 0x01,
	NUM_OF_PIP2_EXECS = 2
} PIP2_Exec;

extern char* PIP2_EXEC_NAMES[NUM_OF_PIP2_EXECS];

typedef enum {
	PIP2_APP_SYS_MODE_BOOT            = 0x00,
	PIP2_APP_SYS_MODE_SCANNING        = 0x01,
	PIP2_APP_SYS_MODE_DEEP_SLEEP      = 0x02,
	PIP2_APP_SYS_MODE_TEST_CONFIG     = 0x03,
	PIP2_APP_SYS_MODE_DEEP_STANDBY    = 0x04,
	NUM_OF_PIP2_APP_SYS_MODES = 5
} PIP2_App_Sys_Mode;

extern char* PIP2_APP_SYS_MODE_NAMES[NUM_OF_PIP2_APP_SYS_MODES];

typedef enum {
	PIP2_IOCTL_CODE_ERASE_FILE         = 0x00,
	PIP2_IOCTL_CODE_SEEK_FILE_POINTERS = 0x01,
	PIP2_IOCTL_CODE_AES_CONTROL        = 0x02,
	PIP2_IOCTL_CODE_FILE_STATS         = 0x03,
	PIP2_IOCTL_CODE_FILE_CRC           = 0x04,

	NUM_OF_PIP2_IOCTL_CODES = 5
} PIP2_IOCTL_Code;

extern char* PIP2_IOCTL_CODE_LABELS[NUM_OF_PIP2_IOCTL_CODES];

#define PIP2_CMD_REG_LSB 0x01
#define PIP2_CMD_REG_MSB 0x01

#define PIP2_PAYLOAD_MIN_LEN 4
#define PIP2_PAYLOAD_MAX_LEN 0xFFFF

typedef struct {
	uint8_t cmd_reg_lsb;                
	uint8_t cmd_reg_msb;                
	uint8_t payload_len_lsb;            
	uint8_t payload_len_msb;            
	struct {                            
		uint8_t seq                : 3;
		uint8_t tag                : 1;
		uint8_t reserved_section_1 : 4;
	} __attribute__((packed));
	struct {                            
		uint8_t cmd_id : 7;
		uint8_t resp   : 1;
	} __attribute__((packed));
} __attribute__((packed)) PIP2_Cmd_Header;

typedef struct {
	uint8_t crc_msb;
	uint8_t crc_lsb;
} __attribute__((packed)) PIP2_Cmd_Footer;

#define PIP2_RSP_MIN_LEN 7

typedef struct {
	uint8_t payload_len_lsb;            
	uint8_t payload_len_msb;            
	struct {                            
		uint8_t seq                : 3;
		uint8_t tag                : 1;
		uint8_t reserved_section_1 : 4;
	} __attribute__((packed));
	struct {                            
		uint8_t cmd_id : 7;
		uint8_t resp   : 1;
	} __attribute__((packed));
	uint8_t status_code;                
} __attribute__((packed)) PIP2_Rsp_Header;

typedef struct {
	uint8_t crc_msb;
	uint8_t crc_lsb;
} __attribute__((packed)) PIP2_Rsp_Footer;

typedef struct {
	PIP2_Cmd_Header header;
	uint8_t file_handle;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_FileClose;

typedef struct {
	PIP2_Rsp_Header header;
	PIP2_Rsp_Footer footer;
} __attribute__((packed)) PIP2_Rsp_Payload_FileClose;

typedef struct {
	PIP2_Cmd_Header header;
	uint8_t file_handle;
	uint8_t ioctl_code;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_FileIOCTL_EraseFile;

typedef struct {
	PIP2_Rsp_Header header;
	PIP2_Rsp_Footer footer;
} __attribute__((packed)) PIP2_Rsp_Payload_FileIOCTL_EraseFile;

typedef struct {
	PIP2_Cmd_Header header;
	uint8_t file_handle;
	uint8_t ioctl_code;
	uint32_t offset;
	uint32_t length;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_FileIOCTL_FileCRC;

typedef struct {
	PIP2_Rsp_Header header;
	uint8_t crc_lsb;
	uint8_t crc_msb;
	PIP2_Rsp_Footer footer;
} __attribute__((packed)) PIP2_Rsp_Payload_FileIOCTL_FileCRC;

typedef struct {
	PIP2_Cmd_Header header;
	uint8_t file_num;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_FileOpen;

typedef struct {
	PIP2_Rsp_Header header;
	uint8_t file_handle;
	PIP2_Rsp_Footer footer;
} __attribute__((packed)) PIP2_Rsp_Payload_FileOpen;

typedef struct {
	PIP2_Cmd_Header header;
	uint8_t file_handle;
	uint8_t read_len_lsb;
	uint8_t read_len_msb;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_FileRead;

typedef struct {
	PIP2_Rsp_Header header;
	uint8_t* data;
	PIP2_Rsp_Footer footer;
} __attribute__((packed)) PIP2_Rsp_Payload_FileRead;

typedef struct {
	PIP2_Cmd_Header header;
	uint8_t file_handle;
	uint8_t* data;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_FileWrite;

#define PIP2_FILE_WRITE_CMD_WITHOUT_DATA_LEN \
	sizeof(PIP2_Cmd_Payload_FileWrite) - sizeof(uint8_t*)

#define PIP2_FILE_WRITE_CMD_MAX_LEN 255

typedef struct {
	PIP2_Rsp_Header header;
	PIP2_Rsp_Footer footer;
} __attribute__((packed)) PIP2_Rsp_Payload_FileWrite;

typedef struct {
	PIP2_Cmd_Header header;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_Reset;

typedef struct {
	PIP2_Cmd_Header header;
	PIP2_Cmd_Footer footer;
} __attribute__((packed)) PIP2_Cmd_Payload_Status;

typedef struct {
	PIP2_Rsp_Header header;             
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
	PIP2_Rsp_Footer footer;
} __attribute__((packed)) PIP2_Rsp_Payload_Status;

extern int (*send_pip2_cmd_via_channel)(const ReportData* report);
extern Poll_Status (*get_pip2_rsp_via_channel)(ReportData* report,
		bool apply_timeout, long double timeout_val);
extern int do_pip2_command(ReportData* cmd, ReportData* rsp);
extern int do_pip2_file_close_cmd(uint8_t seq_num, uint8_t file_handle,
		PIP2_Rsp_Payload_FileClose* rsp);
extern int do_pip2_file_ioctl_erase_file_cmd(uint8_t seq_num,
		uint8_t file_handle, PIP2_Rsp_Payload_FileIOCTL_EraseFile* rsp);
extern int do_pip2_file_ioctl_file_crc_cmd(uint8_t seq_num, uint8_t file_handle,
		uint32_t offset, uint32_t length, PIP2_Rsp_Payload_FileIOCTL_FileCRC* rsp);
extern int do_pip2_file_open_cmd(uint8_t seq_num, uint8_t file_num,
		PIP2_Rsp_Payload_FileOpen* rsp);
extern int do_pip2_file_read_cmd(uint8_t seq_num, uint8_t file_handle,
		uint16_t read_len, PIP2_Rsp_Payload_FileRead* rsp, size_t max_rsp_size);
extern int do_pip2_file_write_cmd(uint8_t seq_num, uint8_t file_handle,
		ByteData* data);
extern int do_pip2_reset_cmd(uint8_t seq_num);
extern int do_pip2_status_cmd(uint8_t seq_num, PIP2_Rsp_Payload_Status* rsp);
extern bool is_pip2_api_active();
extern int setup_pip2_api(ChannelType channel_type, int i2c_bus_arg,
		int i2c_addr_arg);
extern int teardown_pip2_api();

#endif 
