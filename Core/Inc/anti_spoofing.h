/*
 * anti_spoofing.h
 *
 *  Created on: 22 Apr 2026
 *      Author: Iustinian Serban
 */

#ifndef INC_ANTI_SPOOFING_H_
#define INC_ANTI_SPOOFING_H_

#include "navigation_solution.h"

typedef enum {
    GNSS_TRUST_OK = 0,
    GNSS_TRUST_SUSPECT,
    GNSS_TRUST_SPOOFED
} GnssTrustState_t;

typedef struct {
	float last_gnss_speed;
	float last_imu_acc_mag;
	float last_dt_gnss;
	float last_gnss_heading_rad;
	float last_gnss_heading_rate_rad_s;
	float last_imu_yaw_rate_rad_s;
	float last_heading_rate_error_rad_s;
	float stationary_time_s;
	float last_innovation_norm;
	float last_local_east_m;
	float last_local_north_m;
	float last_trusted_speed_mps;

	double last_ecef_x;
	double last_ecef_y;
	double last_ecef_z;
	double ref_lat_deg;
	double ref_lon_deg;
	uint8_t initialized;
	uint8_t ref_initialized;
	uint8_t kf_initialized;

	uint32_t last_imu_tick;

	uint8_t time_jump_flag;
	uint8_t pos_jump_flag;
	uint8_t vel_mismatch_flag;
	uint8_t imu_mismatch_flag;
	uint8_t stale_gnss_flag;
	uint8_t heading_mismatch_flag;
	uint8_t stationary_spoof_flag;
	uint8_t innovation_gate_flag;
	uint8_t gnss_rejected_flag;

	float kf_x[3];      /* [east, north, heading] */
	float kf_p_diag[3]; /* diagonal covariance */

    float risk_score;
    GnssTrustState_t state;
} AntiSpoofingState_t;

void AntiSpoofing_Init(AntiSpoofingState_t *spoofing_state);
void AntiSpoofing_Update(AntiSpoofingState_t *spoofing_state, const NavigationInput_t *navigation_input);

#endif /* INC_ANTI_SPOOFING_H_ */
