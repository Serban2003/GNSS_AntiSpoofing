/*
 * CCSDS_utils.h
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */

#ifndef INC_CCSDS_H_
#define INC_CCSDS_H_

#include "utils.h"

#define CCSDS_HEADER_LEN 16U

typedef struct {
	uint16_t CCSDS_STREAMID; // CCSDS Packet Identification
	uint16_t CCSDS_SEQUENCE; // CCSDS Packet Sequence Control
	uint16_t CCSDS_LENGTH; // CCSDS Packet Data Length
	uint32_t CCSDS_SECONDS; // CCSDS Telemetry Secondary Header (seconds)
	uint16_t CCSDS_SUBSECS; // CCSDS Telemetry Secondary Header (subseconds)
	uint32_t CCSDS_SPARE; // GPS Week Number
} CCSDS_HEADER_t;

typedef enum {
	NOVATEL_OEM615_DATA_TLM = 0x0871, // Packet Identifier for telemetry data of NOVATEL_OEM615
	GENERIC_IMU_DATA_TLM = 0x0926 // Packet Identifier for telemetry data of GENERIC_IMU
} CCSDS_StreamId_t;

StatusCode_t CCSDS_ParseHeader(const uint8_t *buf, uint32_t len, CCSDS_HEADER_t *header);

#endif /* INC_CCSDS_H_ */
