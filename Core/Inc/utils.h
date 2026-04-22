/*
 * utils.h
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include <string.h>

typedef float  float32_t;
typedef double float64_t;

typedef enum
{
	OK = 0,
	ERR_NULL_PTR,
	ERR_LEN_TOO_SMALL,
	ERR_OVERFLOW,
    ERR_INVALID_STREAM_ID,
	ERR_INVALID_PACKET,
	ERR_UNSUPPORTED_PACKET,
	START_PARSING,
	PARSING_DATA
} StatusCode_t;

const char *StatusCodeToString(StatusCode_t status);
uint16_t ParseUint16BE(const uint8_t *buf);
uint32_t ParseUint32BE(const uint8_t *buf);
float32_t ParseFloat32BE(const uint8_t *buf);
float64_t ParseFloat64BE(const uint8_t *buf);

#endif /* INC_UTILS_H_ */
