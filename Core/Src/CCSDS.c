/*
 * CCSDS_utils.c
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */

#include "CCSDS.h"

StatusCode_t CCSDS_ParseHeader(const uint8_t *buf, uint32_t len, CCSDS_HEADER_t *header)
{
    uint32_t index = 0;

	if (!buf || !header) return ERR_NULL_PTR;
    if (len < CCSDS_HEADER_LEN) return ERR_LEN_TOO_SMALL;

    header->CCSDS_STREAMID = ParseUint16BE(&buf[index]); index += 2;
	header->CCSDS_SEQUENCE = ParseUint16BE(&buf[index]); index += 2;
	header->CCSDS_LENGTH = ParseUint16BE(&buf[index]); index += 2;
	header->CCSDS_SECONDS = ParseUint32BE(&buf[index]); index += 4;
	header->CCSDS_SUBSECS = ParseUint16BE(&buf[index]); index += 2;
	header->CCSDS_SPARE = ParseUint32BE(&buf[index]); index += 4;

	return OK;
}
