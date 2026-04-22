/*
 * parser.h
 *
 *  Created on: 22 Apr 2026
 *      Author: Iustinian Serban
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#include <string.h>
#include "utils.h"
#include "CCSDS.h"
#include "NOVATEL_OEM615.h"
#include "GENERIC_IMU.h"
#include "navigation_solution.h"

#define MAX_MSG_LEN 90U

typedef struct {
	volatile uint8_t novatel_oem615_ready;
	volatile uint8_t generic_imu_ready;
} parser_status_t;

uint32_t HAL_GetTick(void);

extern uint8_t g_rx_buffer[MAX_MSG_LEN];
extern uint8_t g_rx_index;
extern uint8_t g_parse_rx;
extern parser_status_t g_parser_status;
extern uint8_t novatel_oem615_frame[NOVATEL_OEM615_DATA_TLM_LEN];
extern uint8_t generic_imu_frame[GENERIC_IMU_DATA_TLM_LEN];

StatusCode_t parseRxData(uint8_t byte);

#endif /* INC_PARSER_H_ */
