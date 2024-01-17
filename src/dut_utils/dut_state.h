/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#ifndef DUT_STATE_H_
#define DUT_STATE_H_

#include "../logging.h"

typedef enum {
	DUT_STATE_INVALID,
	DUT_STATE_TP_FW_BOOT,
	DUT_STATE_TP_FW_SCANNING,
	DUT_STATE_TP_FW_DEEP_SLEEP,
	DUT_STATE_TP_FW_TEST,
	DUT_STATE_TP_FW_DEEP_STANDBY,
	DUT_STATE_TP_FW_PROGRAMMER_IMAGE,
	DUT_STATE_TP_FW_SYS_MODE_ANY,
	DUT_STATE_TP_BL,
	DUT_STATE_AUX_MCU_FW_UTILITY_IMAGE,
	DUT_STATE_AUX_MCU_FW_PROGRAMMER_IMAGE,
	DUT_STATE_DEFAULT,
	NUM_OF_DUT_STATES
} DUT_State;

extern char* DUT_STATE_LABELS[NUM_OF_DUT_STATES];

typedef enum {
	DUT_EXEC_INVALID,
	DUT_EXEC_BL,
	DUT_EXEC_FW,
	NUM_OF_DUT_EXECS
} DUT_Exec;

extern char* DUT_EXEC_LABELS[NUM_OF_DUT_EXECS];

#define BOOT_2_SCANNING_POLLING_INTERVAL_MS         10
#define BOOT_2_SCANNING_MAX_WAIT_MS               2000
#define BOOT_2_SCANNING_INFO_MESSAGE_INTERVAL_MS  1000

extern void set_aux_mcu_active_duration_seconds(uint8_t duration);
extern uint8_t get_aux_mcu_active_duration_seconds();

#endif 
