/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __ADRA_API_BASE_H__
#define __ADRA_API_BASE_H__

#include "base/servo_api_base.h"

class AdraApiBase : private ServoApiBase {
 public:
  AdraApiBase(void);
  AdraApiBase(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id);
  ~AdraApiBase(void);
  void adrainit(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id);
  int connect_net_module(int baud);
  virtual bool is_error(void) = 0;
  virtual void into_usb_pm(void) = 0;

  int get_uuid(int id, char uuid[24]);
  int get_sw_version(int id, char version[12]);
  int get_hw_version(int id, char version[24]);
  int get_multi_version(int id, char version[12]);
  int get_mech_ratio(int id, float* ratio);
  int set_com_id(int id, int set_id);
  int set_com_baud(int id, int baud);
  int reset_err(int id);
  int restart_driver(int id);
  int erase_parm(int id);
  int saved_parm(int id);

  int get_elec_ratio(int id, float* ratio);
  int set_elec_ratio(int id, float ratio);
  int get_motion_dir(int id, uint8_t* dir);
  int set_motion_dir(int id, uint8_t dir);
  int get_iwdg_cyc(int id, int* cyc);
  int set_iwdg_cyc(int id, int cyc);
  int get_temp_limit(int id, int8_t* min, int8_t* max);
  int set_temp_limit(int id, int8_t min, int8_t max);
  int get_volt_limit(int id, uint8_t* min, uint8_t* max);
  int set_volt_limit(int id, uint8_t min, uint8_t max);
  int get_curr_limit(int id, float* curr);
  int set_curr_limit(int id, float curr);
  int get_brake_delay(int id, uint16_t* ontime, uint16_t* offtime);
  int set_brake_delay(int id, uint16_t ontime, uint16_t offtime);

  int get_motion_mode(int id, uint8_t* mode);
  int get_motion_enable(int id, uint8_t* able);
  int get_brake_enable(int id, uint8_t* able);
  int into_motion_mode_pos(int id);
  int into_motion_mode_vel(int id);
  int into_motion_mode_tau(int id);
  int into_motion_enable(int id);
  int into_motion_disable(int id);
  int into_brake_enable(int id);
  int into_brake_disable(int id);

  int get_temp_driver(int id, float* temp);
  int get_temp_motor(int id, float* temp);
  int get_bus_volt(int id, float* volt);
  int get_bus_curr(int id, float* curr);
  int get_multi_volt(int id, float* volt);
  int get_error_code(int id, uint8_t* errcode);

  int get_pos_target(int id, float* pos);
  int set_pos_target(int id, float pos);
  int get_pos_current(int id, float* pos);
  int get_pos_limit_min(int id, float* pos);
  int set_pos_limit_min(int id, float pos);
  int get_pos_limit_max(int id, float* pos);
  int set_pos_limit_max(int id, float pos);
  int get_pos_limit_diff(int id, float* pos);
  int set_pos_limit_diff(int id, float pos);
  int get_pos_pidp(int id, float* p);
  int set_pos_pidp(int id, float p);
  int get_pos_smooth_cyc(int id, uint8_t* cyc);
  int set_pos_smooth_cyc(int id, uint8_t cyc);
  int get_pos_adrc_param(int id, uint8_t i, float* param);
  int set_pos_adrc_param(int id, uint8_t i, float param);
  int pos_cal_zero(int id);

  int get_vel_target(int id, float* vel);
  int set_vel_target(int id, float vel);
  int get_vel_current(int id, float* vel);
  int get_vel_limit_min(int id, float* vel);
  int set_vel_limit_min(int id, float vel);
  int get_vel_limit_max(int id, float* vel);
  int set_vel_limit_max(int id, float vel);
  int get_vel_limit_diff(int id, float* vel);
  int set_vel_limit_diff(int id, float vel);
  int get_vel_pidp(int id, float* p);
  int set_vel_pidp(int id, float p);
  int get_vel_pidi(int id, float* i);
  int set_vel_pidi(int id, float i);
  int get_vel_smooth_cyc(int id, uint8_t* cyc);
  int set_vel_smooth_cyc(int id, uint8_t cyc);
  int get_vel_adrc_param(int id, uint8_t i, float* param);
  int set_vel_adrc_param(int id, uint8_t i, float param);

  int get_tau_target(int id, float* tau);
  int set_tau_target(int id, float tau);
  int get_tau_current(int id, float* tau);
  int get_tau_limit_min(int id, float* tau);
  int set_tau_limit_min(int id, float tau);
  int get_tau_limit_max(int id, float* tau);
  int set_tau_limit_max(int id, float tau);
  int get_tau_limit_diff(int id, float* tau);
  int set_tau_limit_diff(int id, float tau);
  int get_tau_pidp(int id, float* p);
  int set_tau_pidp(int id, float p);
  int get_tau_pidi(int id, float* i);
  int set_tau_pidi(int id, float i);
  int get_tau_smooth_cyc(int id, uint8_t* cyc);
  int set_tau_smooth_cyc(int id, uint8_t cyc);
  int get_tau_adrc_param(int id, uint8_t i, float* param);
  int set_tau_adrc_param(int id, uint8_t i, float param);

  int set_cpos_target(uint8_t sid, uint8_t eid, float* pos);
  int set_ctau_target(uint8_t sid, uint8_t eid, float* tau);
  int set_cpostau_target(uint8_t sid, uint8_t eid, float* pos, float* tau);
  int set_cposvel_target(uint8_t sid, uint8_t eid, float* pos, float* vel);
  int get_spostau_current(uint8_t id, int* num, float* pos, float* tau);
  int get_cpostau_current(uint8_t sid, uint8_t eid, int* num, float* pos, float* tau, int* ret);

 private:
};

class MOTION_MODEL {
 public:
  const static uint8_t POS = 1;
  const static uint8_t VEL = 2;
  const static uint8_t TAU = 3;
};

#endif
