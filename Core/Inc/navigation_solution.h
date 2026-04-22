/*
 * navigation_solution.h
 *
 *  Created on: 22 Apr 2026
 *      Author: IustinianSerban
 */

#ifndef INC_NAVIGATION_SOLUTION_H_
#define INC_NAVIGATION_SOLUTION_H_

#include "NOVATEL_OEM615.h"
#include "GENERIC_IMU.h"

typedef struct {
    NOVATEL_OEM615_tlm_t novatel_oem615_tlm;
    GENERIC_IMU_tlm_t generic_imu_tlm;

    volatile uint32_t novatel_oem615_tick;
    volatile uint32_t generic_imu_tick;
} NavigationInput_t;

extern NavigationInput_t g_navigation_input;

#endif /* INC_NAVIGATION_SOLUTION_H_ */
