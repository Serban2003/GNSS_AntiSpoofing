/*
 * utils.c
 *
 *  Created on: 20 Apr 2026
 *      Author: Iustinian Serban
 */

#include "utils.h"

const char *StatusCodeToString(StatusCode_t status)
{
    switch (status)
    {
        case OK: return "OK";
        case ERR_NULL_PTR: return "Null pointer";
        case ERR_LEN_TOO_SMALL: return "Length too small";
        case ERR_INVALID_STREAM_ID: return "Invalid stream ID";
        case ERR_INVALID_PACKET: return "Invalid packet";
        case ERR_UNSUPPORTED_PACKET: return "Unsupported packet";
        default: return "Unknown error";
    }
}

uint16_t ParseUint16BE(const uint8_t *buf)
{
    return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}

uint32_t ParseUint32BE(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

float32_t ParseFloat32BE(const uint8_t *buf)
{
    uint8_t temp[4];
    float32_t value;

    temp[0] = buf[3];
    temp[1] = buf[2];
    temp[2] = buf[1];
    temp[3] = buf[0];

    memcpy(&value, temp, 4);
    return value;
}

float64_t ParseFloat64BE(const uint8_t *buf)
{
    uint8_t temp[8];
    float64_t value;

    temp[0] = buf[7];
    temp[1] = buf[6];
    temp[2] = buf[5];
    temp[3] = buf[4];
    temp[4] = buf[3];
    temp[5] = buf[2];
    temp[6] = buf[1];
    temp[7] = buf[0];

    memcpy(&value, temp, 8);
    return value;
}
