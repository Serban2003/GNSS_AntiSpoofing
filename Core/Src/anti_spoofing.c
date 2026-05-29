/*
 * anti_spoofing.c
 *
 *  Created on: 22 Apr 2026
 *      Author: Iustinian Serban
 */

#include "anti_spoofing.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EARTH_RADIUS_M                6378137.0

#define GNSS_TIME_JUMP_MAX_S          2.0f
#define GNSS_STALE_MAX_MS             2000U

#define POSITION_JUMP_SPEED_MAX_MPS   20000.0f
#define SPEED_MISMATCH_MAX_MPS        5000.0f

#define SAT_ORBITAL_SPEED_MIN_MPS     6500.0f
#define SAT_ORBITAL_SPEED_MAX_MPS     8500.0f

#define IMU_STATIC_ACC_MAX_MPS2       0.20f
#define IMU_STATIC_GRAVITY_MPS2       9.81f
#define IMU_STATIC_GRAVITY_TOL_MPS2   0.30f
#define IMU_STATIC_RATE_MAX_RAD_S     0.05f

#define HEADING_RATE_ERR_MAX_RAD_S    0.70f
#define HEADING_MIN_TRAVEL_M          0.5f

#define FROZEN_POS_CONFIRM_TIME_S     5.0f
#define FROZEN_POS_EPS_DEG            0.00000001
#define FROZEN_ALT_EPS_M              0.01f
#define FROZEN_GNSS_SPEED_MIN_MPS     1.0f
#define FROZEN_ECEF_SPEED_MIN_MPS     1.0f

#define RISK_TIME_JUMP                0.30f
#define RISK_STALE_GNSS               0.20f
#define RISK_POSITION_JUMP            0.30f
#define RISK_VELOCITY_MISMATCH        0.20f
#define RISK_IMU_MISMATCH             0.25f
#define RISK_HEADING_MISMATCH         0.20f
#define RISK_FROZEN_POSITION          0.35f

#define RISK_SUSPECT_LIMIT            0.30f
#define RISK_SPOOFED_LIMIT            0.65f

#define FLAG_NONE                     0U
#define FLAG_TIME_JUMP                1U
#define FLAG_POSITION_JUMP            2U
#define FLAG_VELOCITY_MISMATCH        3U
#define FLAG_IMU_MISMATCH             4U
#define FLAG_STALE_GNSS               5U
#define FLAG_HEADING_MISMATCH         6U
#define FLAG_FROZEN_POSITION          7U


static float vec3_norm_f(float x, float y, float z) {
    return sqrtf((x * x) + (y * y) + (z * z));
}

static double vec3_dist_d(double x1, double y1, double z1, double x2, double y2, double z2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;

    return sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

static double gnss_time_sec(const NOVATEL_OEM615_tlm_t *telemetry) {
    return ((double)telemetry->GPS_WEEKS * 604800.0) + ((double)telemetry->GPS_SECONDS) + ((double)telemetry->GPS_FRAC_SECS);
}

static float wrap_pi_f(float angle) {
    while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < (float)-M_PI) angle += 2.0f * (float)M_PI;

    return angle;
}

static uint8_t gnss_speed_is_orbital(float gnss_speed) {
    return ((gnss_speed >= SAT_ORBITAL_SPEED_MIN_MPS) && (gnss_speed <= SAT_ORBITAL_SPEED_MAX_MPS));
}

static void reset_flags(AntiSpoofingState_t *spoofing_state) {
    spoofing_state->time_jump_flag = FLAG_NONE;
    spoofing_state->pos_jump_flag = FLAG_NONE;
    spoofing_state->vel_mismatch_flag = FLAG_NONE;
    spoofing_state->imu_mismatch_flag = FLAG_NONE;
    spoofing_state->stale_gnss_flag = FLAG_NONE;
    spoofing_state->heading_mismatch_flag = FLAG_NONE;
    spoofing_state->frozen_position_flag = FLAG_NONE;
    spoofing_state->risk_score = 0.0f;
}

static void update_trust_state(AntiSpoofingState_t *spoofing_state) {
    if (spoofing_state->risk_score >= RISK_SPOOFED_LIMIT) spoofing_state->state = GNSS_TRUST_SPOOFED;
    else if (spoofing_state->risk_score >= RISK_SUSPECT_LIMIT) spoofing_state->state = GNSS_TRUST_SUSPECT;
    else spoofing_state->state = GNSS_TRUST_OK;
}

static uint16_t build_debug_flags(const AntiSpoofingState_t *spoofing_state) {
    if (spoofing_state->frozen_position_flag) return spoofing_state->frozen_position_flag;
    if (spoofing_state->heading_mismatch_flag) return spoofing_state->heading_mismatch_flag;
    if (spoofing_state->stale_gnss_flag) return spoofing_state->stale_gnss_flag;
    if (spoofing_state->imu_mismatch_flag) return spoofing_state->imu_mismatch_flag;
    if (spoofing_state->vel_mismatch_flag) return spoofing_state->vel_mismatch_flag;
    if (spoofing_state->pos_jump_flag) return spoofing_state->pos_jump_flag;
    if (spoofing_state->time_jump_flag) return spoofing_state->time_jump_flag;

    return FLAG_NONE;
}

static void latlon_to_local_en(double lat_deg, double lon_deg, double ref_lat_deg, double ref_lon_deg, float *east_m, float *north_m) {
    double lat_rad = lat_deg * (M_PI / 180.0);
    double lon_rad = lon_deg * (M_PI / 180.0);
    double ref_lat_rad = ref_lat_deg * (M_PI / 180.0);
    double ref_lon_rad = ref_lon_deg * (M_PI / 180.0);

    double d_lat = lat_rad - ref_lat_rad;
    double d_lon = lon_rad - ref_lon_rad;
    double avg_lat = 0.5 * (lat_rad + ref_lat_rad);

    if (east_m) *east_m = (float)(EARTH_RADIUS_M * d_lon * cos(avg_lat));
    if (north_m) *north_m = (float)(EARTH_RADIUS_M * d_lat);
}

static uint8_t imu_is_stationary(float acc_mag, float rate_mag) {
    uint8_t static_linear_acc;
    uint8_t static_gravity_acc;
    uint8_t static_rate;

    static_linear_acc = acc_mag < IMU_STATIC_ACC_MAX_MPS2;
    static_gravity_acc = fabsf(acc_mag - IMU_STATIC_GRAVITY_MPS2) < IMU_STATIC_GRAVITY_TOL_MPS2;
    static_rate = rate_mag < IMU_STATIC_RATE_MAX_RAD_S;

    return (static_rate && (static_linear_acc || static_gravity_acc));
}

static void update_frozen_position_check(AntiSpoofingState_t *spoofing_state, const NOVATEL_OEM615_tlm_t *gnss, double dt_gnss, float gnss_speed, float ecef_speed) {
    uint8_t same_lat;
    uint8_t same_lon;
    uint8_t same_alt;
    uint8_t frozen_pos;

    same_lat = fabs(gnss->LAT - spoofing_state->last_lat_deg) < FROZEN_POS_EPS_DEG;
    same_lon = fabs(gnss->LON - spoofing_state->last_lon_deg) < FROZEN_POS_EPS_DEG;
    same_alt = fabsf(gnss->ALT - spoofing_state->last_alt_m) < FROZEN_ALT_EPS_M;

    frozen_pos = same_lat && same_lon && same_alt;

    if ((dt_gnss > 0.0) && frozen_pos) spoofing_state->frozen_position_time_s += (float)dt_gnss;
    else spoofing_state->frozen_position_time_s = 0.0f;

    if ((spoofing_state->frozen_position_time_s >= FROZEN_POS_CONFIRM_TIME_S) && ((gnss_speed > FROZEN_GNSS_SPEED_MIN_MPS) || (ecef_speed > FROZEN_ECEF_SPEED_MIN_MPS))) {
        spoofing_state->frozen_position_flag = FLAG_FROZEN_POSITION;
        spoofing_state->risk_score += RISK_FROZEN_POSITION;
    }
}

void AntiSpoofing_Init(AntiSpoofingState_t *spoofing_state) {
    if (!spoofing_state) return;

    spoofing_state->last_gnss_speed = 0.0f;
    spoofing_state->last_imu_acc_mag = 0.0f;
    spoofing_state->last_dt_gnss = 0.0f;
    spoofing_state->last_gnss_heading_rad = 0.0f;
    spoofing_state->last_gnss_heading_rate_rad_s = 0.0f;
    spoofing_state->last_imu_yaw_rate_rad_s = 0.0f;
    spoofing_state->last_heading_rate_error_rad_s = 0.0f;
    spoofing_state->stationary_time_s = 0.0f;
    spoofing_state->last_innovation_norm = 0.0f;
    spoofing_state->last_local_east_m = 0.0f;
    spoofing_state->last_local_north_m = 0.0f;
    spoofing_state->last_trusted_speed_mps = 0.0f;

    spoofing_state->last_gnss_time_sec = 0.0;
    spoofing_state->last_ecef_x = 0.0;
    spoofing_state->last_ecef_y = 0.0;
    spoofing_state->last_ecef_z = 0.0;
    spoofing_state->ref_lat_deg = 0.0;
    spoofing_state->ref_lon_deg = 0.0;

    spoofing_state->last_lat_deg = 0.0;
    spoofing_state->last_lon_deg = 0.0;
    spoofing_state->last_alt_m = 0.0f;
    spoofing_state->frozen_position_time_s = 0.0f;

    spoofing_state->initialized = 0U;
    spoofing_state->last_imu_tick = 0U;

    reset_flags(spoofing_state);

    spoofing_state->risk_score = 0.0f;
    spoofing_state->state = GNSS_TRUST_OK;
}

void AntiSpoofing_Update(AntiSpoofingState_t *spoofing_state, const NavigationInput_t *navigation_input) {
    static double prev_gnss_time = 0.0;
    static uint32_t prev_gnss_tick = 0U;

    const NOVATEL_OEM615_tlm_t *gnss;
    const GENERIC_IMU_tlm_t *imu;

    double t_now;
    double dt_gnss;
    double distance_3d;

    uint32_t tick_dt;

    float imu_dt_s = 0.0f;
    float gnss_speed;
    float ecef_speed;
    float imu_acc_mag;
    float imu_rate_mag;
    float imu_yaw_rate;

    float local_east = 0.0f;
    float local_north = 0.0f;
    float d_east;
    float d_north;
    float horizontal_distance;

    float gnss_heading = 0.0f;
    float gnss_heading_rate = 0.0f;
    uint8_t heading_valid = 0U;
    uint8_t orbital_speed = 0U;

    if (!spoofing_state || !navigation_input) return;

    gnss = &navigation_input->novatel_oem615_tlm;
    imu = &navigation_input->generic_imu_tlm;

    reset_flags(spoofing_state);

    gnss_speed = vec3_norm_f((float)gnss->VEL_X, (float)gnss->VEL_Y, (float)gnss->VEL_Z);
    imu_acc_mag = vec3_norm_f(imu->X_LINEAR_ACCELERATION, imu->Y_LINEAR_ACCELERATION, imu->Z_LINEAR_ACCELERATION);
    imu_rate_mag = vec3_norm_f(imu->X_ANGULAR_RATE, imu->Y_ANGULAR_RATE, imu->Z_ANGULAR_RATE);
    imu_yaw_rate = imu->Z_ANGULAR_RATE;

    orbital_speed = gnss_speed_is_orbital(gnss_speed);

    t_now = gnss_time_sec(gnss);
    spoofing_state->last_gnss_time_sec = t_now;

    latlon_to_local_en(gnss->LAT, gnss->LON, spoofing_state->ref_lat_deg, spoofing_state->ref_lon_deg, &local_east, &local_north);

    if (!spoofing_state->initialized) {
        spoofing_state->last_ecef_x = gnss->ECEF_X;
        spoofing_state->last_ecef_y = gnss->ECEF_Y;
        spoofing_state->last_ecef_z = gnss->ECEF_Z;

        spoofing_state->last_local_east_m = local_east;
        spoofing_state->last_local_north_m = local_north;

        spoofing_state->last_gnss_speed = gnss_speed;
        spoofing_state->last_trusted_speed_mps = gnss_speed;

        spoofing_state->last_imu_acc_mag = imu_acc_mag;
        spoofing_state->last_imu_yaw_rate_rad_s = imu_yaw_rate;
        spoofing_state->last_imu_tick = navigation_input->generic_imu_tick;

        spoofing_state->last_lat_deg = gnss->LAT;
        spoofing_state->last_lon_deg = gnss->LON;
        spoofing_state->last_alt_m = gnss->ALT;
        spoofing_state->frozen_position_time_s = 0.0f;

        prev_gnss_time = t_now;
        prev_gnss_tick = navigation_input->novatel_oem615_tick;

        spoofing_state->initialized = 1U;
        return;
    }

    dt_gnss = t_now - prev_gnss_time;
    tick_dt = navigation_input->novatel_oem615_tick - prev_gnss_tick;

    if ((spoofing_state->last_imu_tick != 0U) && (navigation_input->generic_imu_tick >= spoofing_state->last_imu_tick)) {
        imu_dt_s = 0.001f * (float)(navigation_input->generic_imu_tick - spoofing_state->last_imu_tick);
    }

    spoofing_state->last_imu_tick = navigation_input->generic_imu_tick;
    spoofing_state->last_dt_gnss = (float)dt_gnss;

    if ((dt_gnss <= 0.0) || (dt_gnss > GNSS_TIME_JUMP_MAX_S)) {
        spoofing_state->time_jump_flag = FLAG_TIME_JUMP;
        spoofing_state->risk_score += RISK_TIME_JUMP;
    }

    if (tick_dt > GNSS_STALE_MAX_MS) {
        spoofing_state->stale_gnss_flag = FLAG_STALE_GNSS;
        spoofing_state->risk_score += RISK_STALE_GNSS;
    }

    distance_3d = vec3_dist_d(spoofing_state->last_ecef_x, spoofing_state->last_ecef_y, spoofing_state->last_ecef_z, gnss->ECEF_X, gnss->ECEF_Y, gnss->ECEF_Z);

    if (dt_gnss > 0.001) ecef_speed = (float)(distance_3d / dt_gnss);
    else ecef_speed = 0.0f;

    update_frozen_position_check(spoofing_state, gnss, dt_gnss, gnss_speed, ecef_speed);

    if (ecef_speed > POSITION_JUMP_SPEED_MAX_MPS) {
        spoofing_state->pos_jump_flag = FLAG_POSITION_JUMP;
        spoofing_state->risk_score += RISK_POSITION_JUMP;
    }

    if (fabsf(ecef_speed - gnss_speed) > SPEED_MISMATCH_MAX_MPS) {
        spoofing_state->vel_mismatch_flag = FLAG_VELOCITY_MISMATCH;
        spoofing_state->risk_score += RISK_VELOCITY_MISMATCH;
    }

    if (!orbital_speed && (imu_acc_mag < 0.3f) && (fabsf(gnss_speed - spoofing_state->last_gnss_speed) > 10.0f)) {
        spoofing_state->imu_mismatch_flag = FLAG_IMU_MISMATCH;
        spoofing_state->risk_score += RISK_IMU_MISMATCH;
    }

    if (imu_dt_s > 0.0f) {
        if (imu_is_stationary(imu_acc_mag, imu_rate_mag)) spoofing_state->stationary_time_s += imu_dt_s;
        else spoofing_state->stationary_time_s = 0.0f;
    }

    d_east = local_east - spoofing_state->last_local_east_m;
    d_north = local_north - spoofing_state->last_local_north_m;
    horizontal_distance = sqrtf((d_east * d_east) + (d_north * d_north));

    if ((dt_gnss > 0.05) && (horizontal_distance > HEADING_MIN_TRAVEL_M)) {
        gnss_heading = atan2f(d_east, d_north);
        gnss_heading_rate = wrap_pi_f(gnss_heading - spoofing_state->last_gnss_heading_rad) / (float)dt_gnss;

        heading_valid = 1U;
        spoofing_state->last_gnss_heading_rate_rad_s = gnss_heading_rate;
    }

    if (heading_valid) {
        spoofing_state->last_heading_rate_error_rad_s = fabsf(gnss_heading_rate - imu_yaw_rate);

        if (spoofing_state->last_heading_rate_error_rad_s > HEADING_RATE_ERR_MAX_RAD_S) {
            spoofing_state->heading_mismatch_flag = FLAG_HEADING_MISMATCH;
            spoofing_state->risk_score += RISK_HEADING_MISMATCH;
        }
    }

    update_trust_state(spoofing_state);

    spoofing_state->last_ecef_x = gnss->ECEF_X;
    spoofing_state->last_ecef_y = gnss->ECEF_Y;
    spoofing_state->last_ecef_z = gnss->ECEF_Z;

    spoofing_state->last_local_east_m = local_east;
    spoofing_state->last_local_north_m = local_north;

    spoofing_state->last_gnss_speed = gnss_speed;
    spoofing_state->last_imu_acc_mag = imu_acc_mag;
    spoofing_state->last_imu_yaw_rate_rad_s = imu_yaw_rate;

    if (heading_valid) spoofing_state->last_gnss_heading_rad = gnss_heading;

    spoofing_state->last_lat_deg = gnss->LAT;
    spoofing_state->last_lon_deg = gnss->LON;
    spoofing_state->last_alt_m = gnss->ALT;

    prev_gnss_time = t_now;
    prev_gnss_tick = navigation_input->novatel_oem615_tick;
}

void AntiSpoofing_SendDebugUart3(UART_HandleTypeDef *huart, AntiSpoofingState_t *spoofing_state) {
    static uint8_t header_sent = 0U;

    char msg[256];
    uint16_t flags;

    int risk_x100;
    int risk_int;
    int risk_dec;

    uint32_t t_int;
    uint32_t t_dec;

    int frozen_x100;
    int frozen_int;
    int frozen_dec;

    int len;

    if (!huart || !spoofing_state) return;

    if (!header_sent) {
        const char *header = "t,risk,state,main_flag,time,pos,vel,imu,stale,heading,frozen,frozen_time\r\n";
        HAL_UART_Transmit(huart, (uint8_t *)header, (uint16_t)strlen(header), 100);
        header_sent = 1U;
    }

    flags = build_debug_flags(spoofing_state);

    risk_x100 = (int)(spoofing_state->risk_score * 100.0f);
    if (risk_x100 < 0) risk_x100 = 0;

    risk_int = risk_x100 / 100;
    risk_dec = risk_x100 % 100;

    t_int = (uint32_t)spoofing_state->last_gnss_time_sec;
    t_dec = (uint32_t)((spoofing_state->last_gnss_time_sec - (double)t_int) * 1000.0);
    if (t_dec > 999U) t_dec = 999U;

    frozen_x100 = (int)(spoofing_state->frozen_position_time_s * 100.0f);
    if (frozen_x100 < 0) frozen_x100 = 0;

    frozen_int = frozen_x100 / 100;
    frozen_dec = frozen_x100 % 100;

    len = snprintf(msg, sizeof(msg), "%lu.%03lu,%d.%02d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d.%02d\r\n", (unsigned long)t_int, (unsigned long)t_dec, risk_int, risk_dec, (unsigned int)spoofing_state->state, (unsigned int)flags, (unsigned int)spoofing_state->time_jump_flag, (unsigned int)spoofing_state->pos_jump_flag, (unsigned int)spoofing_state->vel_mismatch_flag, (unsigned int)spoofing_state->imu_mismatch_flag, (unsigned int)spoofing_state->stale_gnss_flag, (unsigned int)spoofing_state->heading_mismatch_flag, (unsigned int)spoofing_state->frozen_position_flag, frozen_int, frozen_dec);

    if (len > 0) HAL_UART_Transmit(huart, (uint8_t *)msg, (uint16_t)len, 100);
}
