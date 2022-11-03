/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "adra/adra_api_base.h"

/**
 * Please see the manual: https://umbratek.com/wiki/en/#!adra/adra_api_c.md
 *
 * @param   void         void  [void description]
 *
 * @return  AdraApiBase        [return description]
 */
AdraApiBase::AdraApiBase(void) {}
AdraApiBase::AdraApiBase(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id) : ServoApiBase(bus_type, socket_fp, servo_id) {}
AdraApiBase::~AdraApiBase(void) {}
void AdraApiBase::adrainit(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id) { servoinit(bus_type, socket_fp, servo_id); }
int AdraApiBase::connect_net_module(int baud) { return connect_net(baud); }

int AdraApiBase::get_uuid(int id, char uuid[24]) { return get_uuid_(id, uuid); }
int AdraApiBase::get_sw_version(int id, char version[12]) { return get_sw_version_(id, version); }
int AdraApiBase::get_hw_version(int id, char version[24]) { return get_hw_version_(id, version); }
int AdraApiBase::get_multi_version(int id, char version[12]) { return get_multi_version_(id, version); }
int AdraApiBase::get_mech_ratio(int id, float* ratio) { return get_mech_ratio_(id, ratio); }
int AdraApiBase::set_com_id(int id, int set_id) { return set_com_id_(id, set_id); }
int AdraApiBase::set_com_baud(int id, int baud) { return set_com_baud_(id, baud); }
int AdraApiBase::reset_err(int id) { return reset_err_(id); }
int AdraApiBase::restart_driver(int id) { return restart_driver_(id); }
int AdraApiBase::erase_parm(int id) { return erase_parm_(id); }
int AdraApiBase::saved_parm(int id) { return saved_parm_(id); }

int AdraApiBase::get_elec_ratio(int id, float* ratio) { return get_elec_ratio_(id, ratio); }
int AdraApiBase::set_elec_ratio(int id, float ratio) { return set_elec_ratio_(id, ratio); }
int AdraApiBase::get_motion_dir(int id, uint8_t* dir) { return get_motion_dir_(id, dir); }
int AdraApiBase::set_motion_dir(int id, uint8_t dir) { return set_motion_dir_(id, dir); }
int AdraApiBase::get_iwdg_cyc(int id, int* cyc) { return get_iwdg_cyc_(id, cyc); }
int AdraApiBase::set_iwdg_cyc(int id, int cyc) { return set_iwdg_cyc_(id, cyc); }
int AdraApiBase::get_temp_limit(int id, int8_t* min, int8_t* max) { return get_temp_limit_(id, min, max); }
int AdraApiBase::set_temp_limit(int id, int8_t min, int8_t max) { return set_temp_limit_(id, min, max); }
int AdraApiBase::get_volt_limit(int id, uint8_t* min, uint8_t* max) { return get_volt_limit_(id, min, max); }
int AdraApiBase::set_volt_limit(int id, uint8_t min, uint8_t max) { return set_volt_limit_(id, min, max); }
int AdraApiBase::get_curr_limit(int id, float* curr) { return get_curr_limit_(id, curr); }
int AdraApiBase::set_curr_limit(int id, float curr) { return set_curr_limit_(id, curr); }
int AdraApiBase::get_brake_delay(int id, uint16_t* ontime, uint16_t* offtime) { return get_brake_delay_(id, ontime, offtime); }
int AdraApiBase::set_brake_delay(int id, uint16_t ontime, uint16_t offtime) { return set_brake_delay_(id, ontime, offtime); }

int AdraApiBase::get_motion_mode(int id, uint8_t* mode) { return get_motion_mode_(id, mode); }
int AdraApiBase::get_motion_enable(int id, uint8_t* able) { return get_motion_enable_(id, able); }
int AdraApiBase::get_brake_enable(int id, uint8_t* able) { return get_brake_enable_(id, able); }
int AdraApiBase::into_motion_mode_pos(int id) { return set_motion_mode_(id, MOTION_MODEL::POS); }
int AdraApiBase::into_motion_mode_vel(int id) { return set_motion_mode_(id, MOTION_MODEL::VEL); }
int AdraApiBase::into_motion_mode_tau(int id) { return set_motion_mode_(id, MOTION_MODEL::TAU); }
int AdraApiBase::into_motion_enable(int id) { return set_motion_enable_(id, 1); }
int AdraApiBase::into_motion_disable(int id) { return set_motion_enable_(id, 0); }
int AdraApiBase::into_brake_enable(int id) { return set_brake_enable_(id, 1); }
int AdraApiBase::into_brake_disable(int id) { return set_brake_enable_(id, 0); }

int AdraApiBase::get_temp_driver(int id, float* temp) { return get_temp_driver_(id, temp); }
int AdraApiBase::get_temp_motor(int id, float* temp) { return get_temp_motor_(id, temp); }
int AdraApiBase::get_bus_volt(int id, float* volt) { return get_bus_volt_(id, volt); }
int AdraApiBase::get_bus_curr(int id, float* curr) { return get_bus_curr_(id, curr); }
int AdraApiBase::get_multi_volt(int id, float* volt) { return get_multi_volt_(id, volt); }
int AdraApiBase::get_error_code(int id, uint8_t* errcode) { return get_error_code_(id, errcode); }

int AdraApiBase::get_pos_target(int id, float* pos) { return get_pos_target_(id, pos); }
int AdraApiBase::set_pos_target(int id, float pos) { return set_pos_target_(id, pos); }
int AdraApiBase::get_pos_current(int id, float* pos) { return get_pos_current_(id, pos); }
int AdraApiBase::get_pos_limit_min(int id, float* pos) { return get_pos_limit_min_(id, pos); }
int AdraApiBase::set_pos_limit_min(int id, float pos) { return set_pos_limit_min_(id, pos); }
int AdraApiBase::get_pos_limit_max(int id, float* pos) { return get_pos_limit_max_(id, pos); }
int AdraApiBase::set_pos_limit_max(int id, float pos) { return set_pos_limit_max_(id, pos); }
int AdraApiBase::get_pos_limit_diff(int id, float* pos) { return get_pos_limit_diff_(id, pos); }
int AdraApiBase::set_pos_limit_diff(int id, float pos) { return set_pos_limit_diff_(id, pos); }
int AdraApiBase::get_pos_pidp(int id, float* p) { return get_pos_pidp_(id, p); }
int AdraApiBase::set_pos_pidp(int id, float p) { return set_pos_pidp_(id, p); }
int AdraApiBase::get_pos_smooth_cyc(int id, uint8_t* cyc) { return get_pos_smooth_cyc_(id, cyc); }
int AdraApiBase::set_pos_smooth_cyc(int id, uint8_t cyc) { return set_pos_smooth_cyc_(id, cyc); }
int AdraApiBase::get_pos_adrc_param(int id, uint8_t i, float* param) { return get_pos_adrc_param_(id, i, param); }
int AdraApiBase::set_pos_adrc_param(int id, uint8_t i, float param) { return set_pos_adrc_param_(id, i, param); }
int AdraApiBase::pos_cal_zero(int id) { return pos_cal_zero_(id); }

int AdraApiBase::get_vel_target(int id, float* vel) { return get_vel_target_(id, vel); }
int AdraApiBase::set_vel_target(int id, float vel) { return set_vel_target_(id, vel); }
int AdraApiBase::get_vel_current(int id, float* vel) { return get_vel_current_(id, vel); }
int AdraApiBase::get_vel_limit_min(int id, float* vel) { return get_vel_limit_min_(id, vel); }
int AdraApiBase::set_vel_limit_min(int id, float vel) { return set_vel_limit_min_(id, vel); }
int AdraApiBase::get_vel_limit_max(int id, float* vel) { return get_vel_limit_max_(id, vel); }
int AdraApiBase::set_vel_limit_max(int id, float vel) { return set_vel_limit_max_(id, vel); }
int AdraApiBase::get_vel_limit_diff(int id, float* vel) { return get_vel_limit_diff_(id, vel); }
int AdraApiBase::set_vel_limit_diff(int id, float vel) { return set_vel_limit_diff_(id, vel); }
int AdraApiBase::get_vel_pidp(int id, float* p) { return get_vel_pidp_(id, p); }
int AdraApiBase::set_vel_pidp(int id, float p) { return set_vel_pidp_(id, p); }
int AdraApiBase::get_vel_pidi(int id, float* i) { return get_vel_pidi_(id, i); }
int AdraApiBase::set_vel_pidi(int id, float i) { return set_vel_pidi_(id, i); }
int AdraApiBase::get_vel_smooth_cyc(int id, uint8_t* cyc) { return get_vel_smooth_cyc_(id, cyc); }
int AdraApiBase::set_vel_smooth_cyc(int id, uint8_t cyc) { return set_vel_smooth_cyc_(id, cyc); }
int AdraApiBase::get_vel_adrc_param(int id, uint8_t i, float* param) { return get_vel_adrc_param_(id, i, param); }
int AdraApiBase::set_vel_adrc_param(int id, uint8_t i, float param) { return set_vel_adrc_param_(id, i, param); }

int AdraApiBase::get_tau_target(int id, float* tau) { return get_tau_target_(id, tau); }
int AdraApiBase::set_tau_target(int id, float tau) { return set_tau_target_(id, tau); }
int AdraApiBase::get_tau_current(int id, float* tau) { return get_tau_current_(id, tau); }
int AdraApiBase::get_tau_limit_min(int id, float* tau) { return get_tau_limit_min_(id, tau); }
int AdraApiBase::set_tau_limit_min(int id, float tau) { return set_tau_limit_min_(id, tau); }
int AdraApiBase::get_tau_limit_max(int id, float* tau) { return get_tau_limit_max_(id, tau); }
int AdraApiBase::set_tau_limit_max(int id, float tau) { return set_tau_limit_max_(id, tau); }
int AdraApiBase::get_tau_limit_diff(int id, float* tau) { return get_tau_limit_diff_(id, tau); }
int AdraApiBase::set_tau_limit_diff(int id, float tau) { return set_tau_limit_diff_(id, tau); }
int AdraApiBase::get_tau_pidp(int id, float* p) { return get_tau_pidp_(id, p); }
int AdraApiBase::set_tau_pidp(int id, float p) { return set_tau_pidp_(id, p); }
int AdraApiBase::get_tau_pidi(int id, float* i) { return get_tau_pidi_(id, i); }
int AdraApiBase::set_tau_pidi(int id, float i) { return set_tau_pidi_(id, i); }
int AdraApiBase::get_tau_smooth_cyc(int id, uint8_t* cyc) { return get_tau_smooth_cyc_(id, cyc); }
int AdraApiBase::set_tau_smooth_cyc(int id, uint8_t cyc) { return set_tau_smooth_cyc_(id, cyc); }
int AdraApiBase::get_tau_adrc_param(int id, uint8_t i, float* param) { return get_tau_adrc_param_(id, i, param); }
int AdraApiBase::set_tau_adrc_param(int id, uint8_t i, float param) { return set_tau_adrc_param_(id, i, param); }

int AdraApiBase::set_cpos_target(uint8_t sid, uint8_t eid, float* pos) { return set_cpos_target_(sid, eid, pos); }
int AdraApiBase::set_ctau_target(uint8_t sid, uint8_t eid, float* tau) { return set_ctau_target_(sid, eid, tau); }
int AdraApiBase::set_cpostau_target(uint8_t sid, uint8_t eid, float* pos, float* tau) {
  return set_cpostau_target_(sid, eid, pos, tau);
}
int AdraApiBase::set_cposvel_target(uint8_t sid, uint8_t eid, float* pos, float* vel) {
  return set_cposvel_target_(sid, eid, pos, vel);
}
int AdraApiBase::get_spostau_current(uint8_t id, int* num, float* pos, float* tau) {
  return get_spostau_current(id, num, pos, tau);
}
int AdraApiBase::get_cpostau_current(uint8_t sid, uint8_t eid, int* num, float* pos, float* tau, int* ret) {
  return get_cpostau_current_(sid, eid, num, pos, tau, ret);
}
