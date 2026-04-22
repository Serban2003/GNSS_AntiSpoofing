/*
 * NOVATEL_OEM615.h
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */

#ifndef SRC_NOVATEL_OEM615_H_
#define SRC_NOVATEL_OEM615_H_

#include "utils.h"
#include "CCSDS.h"

#define NOVATEL_OEM615_DATA_TLM_LEN 86U

typedef struct {
	CCSDS_HEADER_t CCSDS_HEADER; // CCSDS Header
	uint32_t GPS_SECONDS; // GPS Seconds into the Week
	float64_t GPS_FRAC_SECS; // GPS Fractions of a Second
	float64_t ECEF_X; // ECEF Position X (meters)
	float64_t ECEF_Y; // ECEF Position Y (meters)
	float64_t ECEF_Z; // ECEF Position Z (meters)
	float64_t VEL_X; // ECEF Velocity X (m/s)
	float64_t VEL_Y; // ECEF Velocity Y (m/s)
	float64_t VEL_Z; // ECEF Velocity Z (m/s)
	float32_t LAT; // Latitude (decimal degrees)
	float32_t LON; // Longitude (decimal degrees)
	float32_t ALT; // Altitude (meters)
} NOVATEL_OEM615_tlm_t;

StatusCode_t NOVATEL_OEM615_ParseTelemetry(const uint8_t *buf, uint32_t len, NOVATEL_OEM615_tlm_t *telemetry);

#endif /* SRC_NOVATEL_OEM615_H_ */
