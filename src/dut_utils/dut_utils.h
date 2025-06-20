/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef PTLIB_DUT_UTILS_DUT_UTILS_H_
#define PTLIB_DUT_UTILS_DUT_UTILS_H_

#include "dut_state.h"
#include "../dut_driver.h"
#include "../pip/fw_bin_header.h"
#include "../pip/pip2.h"
#include "../pip/pip3.h"

#define PRIMARY_FW_BIN_FILE_NUM  0x01
#define MAX_NUM_OF_FILES_TO_ERASE  10

typedef enum {
	FLASH_LOADER_NONE,
	FLASH_LOADER_TP_PROGRAMMER_IMAGE,
	FLASH_LOADER_PIP2_ROM_BL,
	FLASH_LOADER_AUX_MCU_PROGRAMMER_IMAGE,
	NUM_OF_FLASH_LOADERS
} Flash_Loader;

extern char* FW_LOADER_NAMES[NUM_OF_FLASH_LOADERS];


typedef struct {
	Flash_Loader list[NUM_OF_FLASH_LOADERS];
} Flash_Loader_Options;

extern int calibrate_dut();
extern int do_dut_fw_self_test(PIP3_Self_Test_ID self_test_id,
		int output_format_id, ByteData* cmd_params, bool signed_data,
		bool length_known, FW_Self_Test_Results* results);
extern int read_dut_fw_bin_header(FW_Bin_Header* bin_header);
extern int set_dut_state(DUT_State target_state);
extern int write_image_to_dut_flash_file(uint8_t file_num, ByteData* image,
		const ByteData* file_nums_to_erase,
		const Flash_Loader_Options* loader_options);
extern int flash_file_crc(uint8_t file_num, uint32_t offset, uint32_t length, uint16_t* crc);
extern int flash_file_read(uint8_t file_num, uint16_t length, uint8_t* data);
extern int get_data_block(PIP3_Data_Block_ID data_block_id, ByteData* data_block, size_t max_len);

#endif 
