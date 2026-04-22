/*
 * anti_spoofing.c
 *
 *  Created on: 22 Apr 2026
 *      Author: Iustinian Serban
 */

#include "anti_spoofing.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EARTH_RADIUS_M                6378137.0
#define GNSS_TIME_JUMP_MAX_S          2.0f
#define GNSS_STALE_MAX_MS             2000U
#define POSITION_JUMP_SPEED_MAX_MPS   200.0f
#define SPEED_MISMATCH_MAX_MPS        15.0f
#define IMU_STATIC_ACC_MAX_MPS2       0.20f
#define IMU_STATIC_RATE_MAX_RAD_S     0.05f
#define STATIC_CONFIRM_TIME_S         3.0f
#define STATIC_GNSS_MOVE_MAX_MPS      1.5f
#define HEADING_RATE_ERR_MAX_RAD_S    0.70f
#define HEADING_MIN_TRAVEL_M          0.5f
#define KALMAN_GATE_THRESHOLD         25.0f

static float vec3_norm_f(float x, float y, float z)
{
    return sqrtf(x*x + y*y + z*z);
}

static double vec3_dist_d(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

static double gnss_time_sec(const NOVATEL_OEM615_tlm_t *telemetry)
{
    return ((double)telemetry->GPS_WEEKS * 604800.0) + ((double)telemetry->GPS_SECONDS) + ((double)telemetry->GPS_FRAC_SECS);
}

static float wrap_pi_f(float angle)
{
    while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < (float)-M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

void AntiSpoofing_Init(AntiSpoofingState_t *spoofing_state)
{
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

    spoofing_state->last_ecef_x = 0.0;
    spoofing_state->last_ecef_y = 0.0;
    spoofing_state->last_ecef_z = 0.0;
    spoofing_state->ref_lat_deg = 0.0;
    spoofing_state->ref_lon_deg = 0.0;
    spoofing_state->initialized = 0;
    spoofing_state->ref_initialized = 0;
    spoofing_state->kf_initialized = 0;
    spoofing_state->last_imu_tick = 0U;

    spoofing_state->time_jump_flag = 0;
    spoofing_state->pos_jump_flag = 0;
    spoofing_state->vel_mismatch_flag = 0;
    spoofing_state->imu_mismatch_flag = 0;
    spoofing_state->stale_gnss_flag = 0;
    spoofing_state->heading_mismatch_flag = 0;
    spoofing_state->stationary_spoof_flag = 0;
    spoofing_state->innovation_gate_flag = 0;
    spoofing_state->gnss_rejected_flag = 0;

    spoofing_state->kf_x[0] = 0.0f;
    spoofing_state->kf_x[1] = 0.0f;
    spoofing_state->kf_x[2] = 0.0f;
    spoofing_state->kf_p_diag[0] = 25.0f;
    spoofing_state->kf_p_diag[1] = 25.0f;
    spoofing_state->kf_p_diag[2] = 1.0f;

    spoofing_state->risk_score = 0.0f;
    spoofing_state->state = GNSS_TRUST_OK;
}

static void latlon_to_local_en(double lat_deg, double lon_deg,
                               double ref_lat_deg, double ref_lon_deg,
                               float *east_m, float *north_m)
{
    double lat_rad = lat_deg * (M_PI / 180.0);
    double lon_rad = lon_deg * (M_PI / 180.0);
    double ref_lat_rad = ref_lat_deg * (M_PI / 180.0);
    double ref_lon_rad = ref_lon_deg * (M_PI / 180.0);
    double d_lat = lat_rad - ref_lat_rad;
    double d_lon = lon_rad - ref_lon_rad;
    double avg_lat = 0.5 * (lat_rad + ref_lat_rad);

    if (east_m) {
        *east_m = (float)(EARTH_RADIUS_M * d_lon * cos(avg_lat));
    }
    if (north_m) {
        *north_m = (float)(EARTH_RADIUS_M * d_lat);
    }
}

static void kalman_predict_heading_pos(AntiSpoofingState_t *spoofing_state, float dt, float imu_yaw_rate)
{
    float east = spoofing_state->kf_x[0];
    float north = spoofing_state->kf_x[1];
    float heading = spoofing_state->kf_x[2];
    float speed = spoofing_state->last_trusted_speed_mps;

    east += speed * sinf(heading) * dt;
    north += speed * cosf(heading) * dt;
    heading = wrap_pi_f(heading + imu_yaw_rate * dt);

    spoofing_state->kf_x[0] = east;
    spoofing_state->kf_x[1] = north;
    spoofing_state->kf_x[2] = heading;

    spoofing_state->kf_p_diag[0] += 4.0f * dt + 0.5f * speed;
    spoofing_state->kf_p_diag[1] += 4.0f * dt + 0.5f * speed;
    spoofing_state->kf_p_diag[2] += 0.20f * dt + 0.10f * fabsf(imu_yaw_rate);
}

static float kalman_gate_and_update(AntiSpoofingState_t *spoofing_state,
                                    float meas_east,
                                    float meas_north,
                                    float meas_heading,
                                    uint8_t heading_valid)
{
    const float r_pos = 9.0f;
    const float r_heading = 0.20f;
    float innov_e = meas_east - spoofing_state->kf_x[0];
    float innov_n = meas_north - spoofing_state->kf_x[1];
    float innov_h = heading_valid ? wrap_pi_f(meas_heading - spoofing_state->kf_x[2]) : 0.0f;
    float s_e = spoofing_state->kf_p_diag[0] + r_pos;
    float s_n = spoofing_state->kf_p_diag[1] + r_pos;
    float s_h = spoofing_state->kf_p_diag[2] + r_heading;
    float nis = (innov_e * innov_e) / s_e + (innov_n * innov_n) / s_n;

    if (heading_valid) {
        nis += (innov_h * innov_h) / s_h;
    }

    spoofing_state->last_innovation_norm = nis;

    if (nis > KALMAN_GATE_THRESHOLD) {
        spoofing_state->innovation_gate_flag = 1;
        spoofing_state->gnss_rejected_flag = 1;
        return nis;
    }

    {
        float k_e = spoofing_state->kf_p_diag[0] / s_e;
        float k_n = spoofing_state->kf_p_diag[1] / s_n;

        spoofing_state->kf_x[0] += k_e * innov_e;
        spoofing_state->kf_x[1] += k_n * innov_n;
        spoofing_state->kf_p_diag[0] *= (1.0f - k_e);
        spoofing_state->kf_p_diag[1] *= (1.0f - k_n);
    }

    if (heading_valid) {
        float k_h = spoofing_state->kf_p_diag[2] / s_h;
        spoofing_state->kf_x[2] = wrap_pi_f(spoofing_state->kf_x[2] + k_h * innov_h);
        spoofing_state->kf_p_diag[2] *= (1.0f - k_h);
    }

    return nis;
}

void AntiSpoofing_Update(AntiSpoofingState_t *spoofing_state, const NavigationInput_t *navigation_input)
{
    static double prev_gnss_time = 0.0;
    static uint32_t prev_gnss_tick = 0;

    const NOVATEL_OEM615_tlm_t *novatel_oem615_tlm;
    const GENERIC_IMU_tlm_t *generic_imu_tlm;
    double t_now;
    double dt_gnss;
    double distance_3d;
    uint32_t tick_dt;
    float imu_dt_s = 0.0f;
    float gnss_speed_vec;
    float position_speed;
    float horizontal_distance;
    float imu_acc_mag;
    float imu_rate_mag;
    float imu_yaw_rate;
    float local_east = 0.0f;
    float local_north = 0.0f;
    float d_east;
    float d_north;
    float gnss_heading = 0.0f;
    float gnss_heading_rate = 0.0f;
    uint8_t heading_valid = 0;

    if (!spoofing_state || !navigation_input) return;

    novatel_oem615_tlm = &navigation_input->novatel_oem615_tlm;
    generic_imu_tlm = &navigation_input->generic_imu_tlm;

    spoofing_state->time_jump_flag = 0;
    spoofing_state->pos_jump_flag = 0;
    spoofing_state->vel_mismatch_flag = 0;
    spoofing_state->imu_mismatch_flag = 0;
    spoofing_state->stale_gnss_flag = 0;
    spoofing_state->heading_mismatch_flag = 0;
    spoofing_state->stationary_spoof_flag = 0;
    spoofing_state->innovation_gate_flag = 0;
    spoofing_state->gnss_rejected_flag = 0;
    spoofing_state->risk_score = 0.0f;

    gnss_speed_vec = vec3_norm_f((float)novatel_oem615_tlm->VEL_X, (float)novatel_oem615_tlm->VEL_Y, (float)novatel_oem615_tlm->VEL_Z);
    imu_acc_mag = vec3_norm_f(generic_imu_tlm->X_LINEAR_ACCELERATION,
                              generic_imu_tlm->Y_LINEAR_ACCELERATION,
                              generic_imu_tlm->Z_LINEAR_ACCELERATION);
    imu_rate_mag = vec3_norm_f(generic_imu_tlm->X_ANGULAR_RATE,
                               generic_imu_tlm->Y_ANGULAR_RATE,
                               generic_imu_tlm->Z_ANGULAR_RATE);
    imu_yaw_rate = generic_imu_tlm->Z_ANGULAR_RATE;

    t_now = gnss_time_sec(novatel_oem615_tlm);

    if (!spoofing_state->ref_initialized) {
        spoofing_state->ref_lat_deg = novatel_oem615_tlm->LAT;
        spoofing_state->ref_lon_deg = novatel_oem615_tlm->LON;
        spoofing_state->ref_initialized = 1;
    }

    latlon_to_local_en(novatel_oem615_tlm->LAT, novatel_oem615_tlm->LON, spoofing_state->ref_lat_deg, spoofing_state->ref_lon_deg, &local_east, &local_north);

    if (!spoofing_state->initialized)
    {
        spoofing_state->last_ecef_x = novatel_oem615_tlm->ECEF_X;
        spoofing_state->last_ecef_y = novatel_oem615_tlm->ECEF_Y;
        spoofing_state->last_ecef_z = novatel_oem615_tlm->ECEF_Z;
        spoofing_state->last_local_east_m = local_east;
        spoofing_state->last_local_north_m = local_north;
        spoofing_state->last_gnss_speed = gnss_speed_vec;
        spoofing_state->last_trusted_speed_mps = gnss_speed_vec;
        spoofing_state->last_imu_acc_mag = imu_acc_mag;
        spoofing_state->last_imu_yaw_rate_rad_s = imu_yaw_rate;
        spoofing_state->last_imu_tick = navigation_input->generic_imu_tick;
        prev_gnss_time = t_now;
        prev_gnss_tick = navigation_input->novatel_oem615_tick;
        spoofing_state->initialized = 1;
        return;
    }

    dt_gnss = t_now - prev_gnss_time;
    tick_dt = navigation_input->novatel_oem615_tick - prev_gnss_tick;

    if (spoofing_state->last_imu_tick != 0U && navigation_input->generic_imu_tick >= spoofing_state->last_imu_tick) {
        imu_dt_s = 0.001f * (float)(navigation_input->generic_imu_tick - spoofing_state->last_imu_tick);
    }
    spoofing_state->last_imu_tick = navigation_input->generic_imu_tick;

    spoofing_state->last_dt_gnss = (float)dt_gnss;

    if (dt_gnss <= 0.0 || dt_gnss > GNSS_TIME_JUMP_MAX_S) {
        spoofing_state->time_jump_flag = 1;
        spoofing_state->risk_score += 0.30f;
    }

    if (tick_dt > GNSS_STALE_MAX_MS) {
        spoofing_state->stale_gnss_flag = 1;
        spoofing_state->risk_score += 0.20f;
    }

    distance_3d = vec3_dist_d(spoofing_state->last_ecef_x, spoofing_state->last_ecef_y, spoofing_state->last_ecef_z,
                              novatel_oem615_tlm->ECEF_X, novatel_oem615_tlm->ECEF_Y, novatel_oem615_tlm->ECEF_Z);

    position_speed = (dt_gnss > 0.001) ? (float)(distance_3d / dt_gnss) : 0.0f;

    if (position_speed > POSITION_JUMP_SPEED_MAX_MPS) {
        spoofing_state->pos_jump_flag = 1;
        spoofing_state->risk_score += 0.30f;
    }

    if (fabsf(position_speed - gnss_speed_vec) > SPEED_MISMATCH_MAX_MPS) {
        spoofing_state->vel_mismatch_flag = 1;
        spoofing_state->risk_score += 0.20f;
    }

    if (imu_acc_mag < 0.3f && fabsf(gnss_speed_vec - spoofing_state->last_gnss_speed) > 10.0f) {
        spoofing_state->imu_mismatch_flag = 1;
        spoofing_state->risk_score += 0.25f;
    }

    if (imu_dt_s > 0.0f) {
        if ((imu_acc_mag < IMU_STATIC_ACC_MAX_MPS2) && (imu_rate_mag < IMU_STATIC_RATE_MAX_RAD_S)) {
            spoofing_state->stationary_time_s += imu_dt_s;
        } else {
            spoofing_state->stationary_time_s = 0.0f;
        }
    }

    d_east = local_east - spoofing_state->last_local_east_m;
    d_north = local_north - spoofing_state->last_local_north_m;
    horizontal_distance = sqrtf(d_east * d_east + d_north * d_north);

    if ((dt_gnss > 0.05) && (horizontal_distance > HEADING_MIN_TRAVEL_M)) {
        gnss_heading = atan2f(d_east, d_north);
        gnss_heading_rate = wrap_pi_f(gnss_heading - spoofing_state->last_gnss_heading_rad) / (float)dt_gnss;
        heading_valid = 1;
        spoofing_state->last_gnss_heading_rate_rad_s = gnss_heading_rate;
    }

    if (heading_valid) {
        spoofing_state->last_heading_rate_error_rad_s = fabsf(gnss_heading_rate - imu_yaw_rate);
        if (spoofing_state->last_heading_rate_error_rad_s > HEADING_RATE_ERR_MAX_RAD_S) {
            spoofing_state->heading_mismatch_flag = 1;
            spoofing_state->risk_score += 0.20f;
        }
    }

    if ((spoofing_state->stationary_time_s >= STATIC_CONFIRM_TIME_S) &&
        ((gnss_speed_vec > STATIC_GNSS_MOVE_MAX_MPS) ||
         ((dt_gnss > 0.05) && ((horizontal_distance / (float)dt_gnss) > STATIC_GNSS_MOVE_MAX_MPS)))) {
        spoofing_state->stationary_spoof_flag = 1;
        spoofing_state->risk_score += 0.35f;
    }

    if (!spoofing_state->kf_initialized) {
        spoofing_state->kf_x[0] = local_east;
        spoofing_state->kf_x[1] = local_north;
        spoofing_state->kf_x[2] = heading_valid ? gnss_heading : 0.0f;
        spoofing_state->kf_p_diag[0] = 25.0f;
        spoofing_state->kf_p_diag[1] = 25.0f;
        spoofing_state->kf_p_diag[2] = 1.0f;
        spoofing_state->kf_initialized = 1;
    } else {
        float dt_pred = (dt_gnss > 0.001) ? (float)dt_gnss : imu_dt_s;
        if (dt_pred > 0.0f) {
            kalman_predict_heading_pos(spoofing_state, dt_pred, imu_yaw_rate);
            kalman_gate_and_update(spoofing_state, local_east, local_north, gnss_heading, heading_valid);

            if (spoofing_state->innovation_gate_flag) {
                spoofing_state->risk_score += 0.30f;
            } else {
                spoofing_state->last_trusted_speed_mps = gnss_speed_vec;
            }
        }
    }

    if (spoofing_state->risk_score >= 0.65f) spoofing_state->state = GNSS_TRUST_SPOOFED;
    else if (spoofing_state->risk_score >= 0.30f) spoofing_state->state = GNSS_TRUST_SUSPECT;
    else spoofing_state->state = GNSS_TRUST_OK;

    spoofing_state->last_ecef_x = novatel_oem615_tlm->ECEF_X;
    spoofing_state->last_ecef_y = novatel_oem615_tlm->ECEF_Y;
    spoofing_state->last_ecef_z = novatel_oem615_tlm->ECEF_Z;
    spoofing_state->last_local_east_m = local_east;
    spoofing_state->last_local_north_m = local_north;
    spoofing_state->last_gnss_speed = gnss_speed_vec;
    spoofing_state->last_imu_acc_mag = imu_acc_mag;
    spoofing_state->last_imu_yaw_rate_rad_s = imu_yaw_rate;
    if (heading_valid) {
        spoofing_state->last_gnss_heading_rad = gnss_heading;
    }
    prev_gnss_time = t_now;
    prev_gnss_tick = navigation_input->novatel_oem615_tick;
}


