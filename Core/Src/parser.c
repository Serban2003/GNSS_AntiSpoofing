/*
 * parser.c
 *
 *  Created on: 22 Apr 2026
 *      Author: Iustinian Serban
 */

#include "parser.h"

uint8_t g_rx_buffer[MAX_MSG_LEN];
uint8_t g_rx_index = 0;
uint8_t g_parse_rx = 0;

uint8_t novatel_oem615_frame[NOVATEL_OEM615_DATA_TLM_LEN];
uint8_t generic_imu_frame[GENERIC_IMU_DATA_TLM_LEN];

parser_status_t g_parser_status = {0};

StatusCode_t parseRxData(uint8_t byte)
{
    if (!g_parse_rx){
        if (byte == NOVATEL_OEM615_DATA_TLM_P1 || byte == GENERIC_IMU_DATA_TLM_P1){
            g_parse_rx = 1;
            g_rx_index = 0;
            g_rx_buffer[g_rx_index++] = byte;
            return START_PARSING;
        }
        return ERR_UNSUPPORTED_PACKET;
    }

    if (g_rx_index < MAX_MSG_LEN){
        g_rx_buffer[g_rx_index++] = byte;
    }
    else{
        g_parse_rx = 0;
        g_rx_index = 0;
        return ERR_OVERFLOW;
    }

    if (g_rx_index >= 2){
        if (g_rx_buffer[0] == NOVATEL_OEM615_DATA_TLM_P1 && g_rx_buffer[1] == NOVATEL_OEM615_DATA_TLM_P2){
            if (g_rx_index == NOVATEL_OEM615_DATA_TLM_LEN){
                g_parse_rx = 0;

                memcpy(novatel_oem615_frame, g_rx_buffer, NOVATEL_OEM615_DATA_TLM_LEN);
                g_navigation_input.novatel_oem615_tick = HAL_GetTick();
                g_parser_status.novatel_oem615_ready = 1;
                return OK;
            }
            return PARSING_DATA;
        }

        if (g_rx_buffer[0] == GENERIC_IMU_DATA_TLM_P1 && g_rx_buffer[1] == GENERIC_IMU_DATA_TLM_P2){
            if (g_rx_index == GENERIC_IMU_DATA_TLM_LEN){
                g_parse_rx = 0;

                memcpy(generic_imu_frame, g_rx_buffer, GENERIC_IMU_DATA_TLM_LEN);
                g_navigation_input.generic_imu_tick = HAL_GetTick();
                g_parser_status.generic_imu_ready = 1;
                return OK;
            }
            return PARSING_DATA;
        }

        /* false start */
        g_parse_rx = 0;
        g_rx_index = 0;
        return ERR_INVALID_PACKET;
    }

    return ERR_UNSUPPORTED_PACKET;
}

