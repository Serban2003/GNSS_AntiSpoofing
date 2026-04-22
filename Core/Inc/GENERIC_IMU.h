/*
 * GENERIC_IMU.h
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */

#ifndef INC_GENERIC_IMU_H_
#define INC_GENERIC_IMU_H_

#include "utils.h"
#include "CCSDS.h"

#define GENERIC_IMU_DATA_TLM_LEN 40U

typedef struct {
	CCSDS_HEADER_t CCSDS_HEADER; // CCSDS Header
	float32_t X_LINEAR_ACCELERATION; // Linear acceleration in the X-direction (m/s^2)
	float32_t X_ANGULAR_RATE; // Angular rate in the X-direction (rad/s)
	float32_t Y_LINEAR_ACCELERATION; // Linear acceleration in the Y-direction (m/s^2)
	float32_t Y_ANGULAR_RATE; // Angular rate in the Y-direction (rad/s)
	float32_t Z_LINEAR_ACCELERATION; // Linear acceleration in the Z-direction (m/s^2)
	float32_t Z_ANGULAR_RATE; // Angular rate in the Z-direction (rad/s)
} GENERIC_IMU_tlm_t;

StatusCode_t GENERIC_IMU_ParseTelemetry(const uint8_t *buf, uint32_t len, GENERIC_IMU_tlm_t *telemetry);

#endif /* INC_GENERIC_IMU_H_ */
