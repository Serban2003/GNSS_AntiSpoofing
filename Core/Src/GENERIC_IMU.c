/*
 * GENERIC_IMU.c
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */
#include "GENERIC_IMU.h"

StatusCode_t GENERIC_IMU_ParseTelemetry(const uint8_t *buf, uint32_t len, GENERIC_IMU_tlm_t *telemetry)
{
    uint32_t index;

    if (!buf || !telemetry) return ERR_NULL_PTR;
    if (len < GENERIC_IMU_DATA_TLM_LEN) return ERR_LEN_TOO_SMALL;

    StatusCode_t ret = CCSDS_ParseHeader(buf, len, &telemetry->CCSDS_HEADER);
    if (ret != OK) return ret;

    if (telemetry->CCSDS_HEADER.CCSDS_STREAMID != GENERIC_IMU_DATA_TLM)
        return ERR_INVALID_STREAM_ID;

    index = CCSDS_HEADER_LEN;

    telemetry->X_LINEAR_ACCELERATION = ParseFloat32BE(&buf[index]); index += 4;
    telemetry->X_ANGULAR_RATE        = ParseFloat32BE(&buf[index]); index += 4;
    telemetry->Y_LINEAR_ACCELERATION = ParseFloat32BE(&buf[index]); index += 4;
    telemetry->Y_ANGULAR_RATE        = ParseFloat32BE(&buf[index]); index += 4;
    telemetry->Z_LINEAR_ACCELERATION = ParseFloat32BE(&buf[index]); index += 4;
    telemetry->Z_ANGULAR_RATE        = ParseFloat32BE(&buf[index]); index += 4;

    return OK;
}
