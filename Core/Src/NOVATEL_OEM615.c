/*
 * NOVATEL_OEM615.c
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */

#include "NOVATEL_OEM615.h"

StatusCode_t NOVATEL_OEM615_ParseTelemetry(const uint8_t *buf, uint32_t len, NOVATEL_OEM615_tlm_t *telemetry)
{
	uint32_t index = 0;

	if (!buf || !telemetry) return ERR_NULL_PTR;
	if (len < NOVATEL_OEM615_DATA_TLM_LEN) return ERR_LEN_TOO_SMALL;

	StatusCode_t ret = CCSDS_ParseHeader(buf, len, &telemetry->CCSDS_HEADER);
	if (ret != OK) return ret;

	if (telemetry->CCSDS_HEADER.CCSDS_STREAMID != NOVATEL_OEM615_DATA_TLM)
		return ERR_INVALID_STREAM_ID;

	index = CCSDS_HEADER_LEN;
	telemetry->GPS_SECONDS   = ParseUint32BE(&buf[index]);  index += 4;
	telemetry->GPS_FRAC_SECS = ParseFloat64BE(&buf[index]); index += 8;

	telemetry->ECEF_X = ParseFloat64BE(&buf[index]); index += 8;
	telemetry->ECEF_Y = ParseFloat64BE(&buf[index]); index += 8;
	telemetry->ECEF_Z = ParseFloat64BE(&buf[index]); index += 8;

	telemetry->VEL_X = ParseFloat64BE(&buf[index]); index += 8;
	telemetry->VEL_Y = ParseFloat64BE(&buf[index]); index += 8;
	telemetry->VEL_Z = ParseFloat64BE(&buf[index]); index += 8;

	telemetry->LAT = ParseFloat32BE(&buf[index]); index += 4;
	telemetry->LON = ParseFloat32BE(&buf[index]); index += 4;
	telemetry->ALT = ParseFloat32BE(&buf[index]); index += 4;

    return OK;
}
