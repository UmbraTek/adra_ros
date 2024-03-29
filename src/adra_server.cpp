/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include <stdio.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <string>
#include <thread>
#include "adra/adra_api_serial.h"
#include "adra/adra_api_tcp.h"
#include "adra/adra_api_udp.h"
#include "adra_ros/AdraMsg.h"
#include "adra_ros/Api.h"
#include "ros/ros.h"

static AdraApiBase *adra = NULL;
static ros::Publisher chatter_pub;

constexpr unsigned int hash(const char *s, int off = 0) { return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off]; }

static void connect_to_adra(adra_ros::Api::Request &req, adra_ros::Api::Response &res, int type) {
  if (req.args.size() < 3) {
    res.rets.push_back("-1");
    res.rets.push_back("args's count must 3");
    return;
  }
  if (adra != NULL) {
    res.rets.push_back("0");
    res.rets.push_back("server have connected adra");
    return;
  }
  int id = std::stoi(req.args[1].c_str());
  long baud = std::stoi(req.args[2].c_str());
  const char *ip = req.args[0].c_str();
  const char *com = req.args[0].c_str();
  ROS_INFO("[AdraServ] connect device:%s baud:%ld", req.args[0].c_str(), baud);

  if (1 == type || 2 == type)
    adra = new AdraApiSerial(com, baud);
  else if (3 == type)
    adra = new AdraApiTcp((char *)ip, 6001, 0, 1, 5001, baud);
  else if (4 == type)
    adra = new AdraApiUdp((char *)ip, 5001, 0, 1, 6001, baud);

  uint8_t mode;
  int ret = adra->get_motion_mode(id, &mode);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("connect failed!");
    return;
  } else {
    res.rets.push_back("0");
    res.rets.push_back("connect seccess!");
  }
}

static void connect_serial_rs485(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  connect_to_adra(req, res, 1);
}

static void connect_tcp_rs485(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  connect_to_adra(req, res, 3);
}

static void connect_udp_rs485(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  connect_to_adra(req, res, 4);
}

static void connect(adra_ros::Api::Request &req, adra_ros::Api::Response &res) { connect_serial_rs485(req, res); }

static void disconnect(adra_ros::Api::Response &res) {
  if (adra != NULL) {
    delete adra;
    adra = NULL;
    res.rets.push_back("0");
    res.rets.push_back("disconnect seccess!");
    return;
  }
  res.rets.push_back("-1");
  res.rets.push_back("server have not connect adra!");
}

static bool check_connect(adra_ros::Api::Response &res) {
  if (adra == NULL) {
    res.rets.push_back("-1");
    res.rets.push_back("server have not connect adra");
    return false;
  } else {
    return true;
  }
}

static bool check_args_count(adra_ros::Api::Request &req, adra_ros::Api::Response &res, int count) {
  if (req.args.size() < count) {
    res.rets.push_back("-1");
    char str[25];
    sprintf(str, "args's count must, %d", count);
    res.rets.push_back(str);
    return false;
  } else {
    return true;
  }
}

/**********************************************************************************************************************
 *
 *
 *
 *********************************************************************************************************************/
static std::string char_to_str(char *ch) {
  std::string a(ch);
  return a;
}

static void get_uuid(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  char uuid[24];
  int ret = adra->get_uuid(id, uuid);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(char_to_str(uuid));
}

static void get_sw_version(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  char version[12];
  int ret = adra->get_sw_version(id, version);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(char_to_str(version));
}

static void get_hw_version(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  char version[24];
  int ret = adra->get_hw_version(id, version);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(char_to_str(version));
}

static void get_multi_version(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  char version[12];
  int ret = adra->get_multi_version(id, version);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(char_to_str(version));
}

static void get_mech_ratio(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float ratio;
  int ret = adra->get_mech_ratio(id, &ratio);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(ratio));
}

void set_com_id(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 2) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int temp = std::stoi(req.args[1].c_str());
  int ret = adra->set_com_id(id, temp);
  res.rets.push_back(std::to_string(ret));
}

void set_com_baud(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 2) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int temp = std::stoi(req.args[1].c_str());
  int ret = adra->set_com_baud(id, temp);
  res.rets.push_back(std::to_string(ret));
}

static void reset_err(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->reset_err(id);
  res.rets.push_back(std::to_string(ret));
}

static void restart_driver(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->restart_driver(id);
  res.rets.push_back(std::to_string(ret));
}

static void erase_parm(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->erase_parm(id);
  res.rets.push_back(std::to_string(ret));
}
static void saved_parm(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->saved_parm(id);
  res.rets.push_back(std::to_string(ret));
}

/**********************************************************************************************************************
 *
 *
 *
 *********************************************************************************************************************/
static void get_error_code(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  uint8_t errcode = -1;
  int ret = adra->get_error_code(id, &errcode);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(errcode));
}

static void into_motion_mode_pos(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->into_motion_mode_pos(id);
  res.rets.push_back(std::to_string(ret));
}
static void into_motion_mode_vel(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->into_motion_mode_vel(id);
  res.rets.push_back(std::to_string(ret));
}
static void into_motion_mode_tau(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->into_motion_mode_tau(id);
  res.rets.push_back(std::to_string(ret));
}
static void get_motion_mode(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  uint8_t mode = -1;
  int ret = adra->get_motion_mode(id, &mode);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(mode));
  switch (mode) {
    case 1:
      res.rets.push_back("current is position mode");
      return;
    case 2:
      res.rets.push_back("current is velicity mode");
      return;
    case 3:
      res.rets.push_back("current is current mode");
      return;
    default:
      res.rets.push_back("can not connect the adra");
      return;
  }
}

void into_motion_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->into_motion_enable(id);
  res.rets.push_back(std::to_string(ret));
}
void into_motion_disable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->into_motion_disable(id);
  res.rets.push_back(std::to_string(ret));
}
void get_motion_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  uint8_t able = -1;
  int ret = adra->get_motion_enable(id, &able);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(able));
  switch (able) {
    case 0:
      res.rets.push_back("motion is disable");
      return;
    case 1:
      res.rets.push_back("motion is enable");
      return;
    default:
      res.rets.push_back("can not connect the adra");
      return;
  }
}

void into_brake_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->into_brake_enable(id);
  res.rets.push_back(std::to_string(ret));
}
void into_brake_disable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->into_brake_disable(id);
  res.rets.push_back(std::to_string(ret));
}
void get_brake_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  uint8_t able = -1;
  int ret = adra->get_brake_enable(id, &able);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(able));
  switch (able) {
    case 0:
      res.rets.push_back("brake is disable");
      return;
    case 1:
      res.rets.push_back("brake is enable");
      return;
    default:
      res.rets.push_back("can not connect the adra");
      return;
  }
}

void set_pos_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 2) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float pos = std::stof(req.args[1].c_str());
  int ret = adra->set_pos_target(id, pos);
  res.rets.push_back(std::to_string(ret));
}
void get_pos_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float pos = -1;
  int ret = adra->get_pos_target(id, &pos);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(pos));
}
void get_pos_current(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float pos = -1;
  int ret = adra->get_pos_current(id, &pos);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(pos));
}
void pos_cal_zero(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  int ret = adra->pos_cal_zero(id);
  res.rets.push_back(std::to_string(ret));
}

void set_vel_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 2) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float vel = std::stof(req.args[1].c_str());
  int ret = adra->set_vel_target(id, vel);
  res.rets.push_back(std::to_string(ret));
}
void get_vel_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float vel = -1;
  int ret = adra->get_vel_target(id, &vel);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(vel));
}
void get_vel_current(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float vel = -1;
  int ret = adra->get_vel_current(id, &vel);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(vel));
}
void set_vel_limit_min(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 2) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float vel = std::stof(req.args[1].c_str());
  int ret = adra->set_vel_limit_min(id, vel);
  res.rets.push_back(std::to_string(ret));
}
void get_vel_limit_min(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float vel = -1;
  int ret = adra->get_vel_limit_min(id, &vel);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(vel));
}
void set_vel_limit_max(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 2) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float vel = std::stof(req.args[1].c_str());
  int ret = adra->set_vel_limit_max(id, vel);
  res.rets.push_back(std::to_string(ret));
}
void get_vel_limit_max(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float vel = -1;
  int ret = adra->get_vel_limit_max(id, &vel);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(vel));
}

void set_tau_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 2) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float tau = std::stof(req.args[1].c_str());
  int ret = adra->set_tau_target(id, tau);
  res.rets.push_back(std::to_string(ret));
}
void get_tau_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float tau = -1;
  int ret = adra->get_tau_target(id, &tau);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(tau));
}
void get_tau_current(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_args_count(req, res, 1) == false) return;
  int id = std::stoi(req.args[0].c_str());
  float tau = -1;
  int ret = adra->get_tau_current(id, &tau);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(tau));
}

bool api(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  ROS_INFO("[AdraServ] request: api_name=%s", req.api_name.c_str());
  if (check_connect(res) == false) return false;

  switch (hash(req.api_name.c_str())) {
    case hash("connect"):
      connect(req, res);
      break;
    case hash("disconnect"):
      disconnect(res);
      break;

    case hash("get_uuid"):
      get_uuid(req, res);
      break;
    case hash("get_sw_version"):
      get_sw_version(req, res);
      break;
    case hash("get_hw_version"):
      get_hw_version(req, res);
      break;
    case hash("get_multi_version"):
      get_multi_version(req, res);
      break;
    case hash("get_mech_ratio"):
      get_mech_ratio(req, res);
      break;
    case hash("set_com_baud"):
      set_com_baud(req, res);
      break;
    case hash("reset_err"):
      reset_err(req, res);
      break;
    case hash("restart_driver"):
      restart_driver(req, res);
      break;
    case hash("erase_parm"):
      erase_parm(req, res);
      break;
    case hash("saved_parm"):
      saved_parm(req, res);
      break;

    case hash("get_error_code"):
      get_error_code(req, res);
      break;

    case hash("into_motion_mode_pos"):
      into_motion_mode_pos(req, res);
      break;
    case hash("into_motion_mode_vel"):
      into_motion_mode_vel(req, res);
      break;
    case hash("into_motion_mode_tau"):
      into_motion_mode_tau(req, res);
      break;
    case hash("get_motion_mode"):
      get_motion_mode(req, res);
      break;

    case hash("into_motion_enable"):
      into_motion_enable(req, res);
      break;
    case hash("into_motion_disable"):
      into_motion_disable(req, res);
      break;
    case hash("get_motion_enable"):
      get_motion_enable(req, res);
      break;

    case hash("into_brake_enable"):
      into_brake_enable(req, res);
      break;
    case hash("into_brake_disable"):
      into_brake_disable(req, res);
      break;
    case hash("get_brake_enable"):
      get_brake_enable(req, res);
      break;

    case hash("set_pos_target"):
      set_pos_target(req, res);
      break;
    case hash("get_pos_target"):
      get_pos_target(req, res);
      break;
    case hash("get_pos_current"):
      get_pos_current(req, res);
      break;
    case hash("pos_cal_zero"):
      pos_cal_zero(req, res);
      break;

    case hash("set_vel_target"):
      set_vel_target(req, res);
      break;
    case hash("get_vel_target"):
      get_vel_target(req, res);
      break;
    case hash("get_vel_current"):
      get_vel_current(req, res);
      break;
    case hash("set_vel_limit_min"):
      set_vel_limit_min(req, res);
      break;
    case hash("get_vel_limit_min"):
      get_vel_limit_min(req, res);
      break;
    case hash("set_vel_limit_max"):
      set_vel_limit_max(req, res);
      break;
    case hash("get_vel_limit_max"):
      get_vel_limit_max(req, res);
      break;

    case hash("set_tau_target"):
      set_tau_target(req, res);
      break;
    case hash("get_tau_target"):
      get_tau_target(req, res);
      break;
    case hash("get_tau_current"):
      get_tau_current(req, res);
      break;
    default:
      break;
  }
  return true;
}

void publishThread(int id) {
  while (ros::ok()) {
    adra_ros::AdraMsg msg;
    msg.id = id;
    int ret = adra->get_pos_current(id, &msg.position);
    ret = adra->get_vel_current(id, &msg.velocity);
    ret = adra->get_tau_current(id, &msg.current);
    chatter_pub.publish(msg);

    usleep(100);
  }
}

/**
 *
 * device: PC physical connection interface
 *         1: USB-To-RS485/CAN /dev/ttyUSBx")
 *         2: USB-To-RS485/CAN /dev/ttyACMx")
 *         3: EtherNet-To-RS485/CAN TCP")
 *         4: EtherNet-To-RS485/CAN UDP")
 *         5: PCIe-To-RS485/CAN /dev/ttyUT0")
 *
 * bus:    Actuator interface
 *         0: RS485")
 *         1: CAN")
 *
 * port:   com or ip
 *         [/dev/ttyUSBx] or [/dev/ttyACMx] or [192.168.1.168]
 *
 * baud:   The communication baud rate of the actuator
 * id:     The communication id rate of the actuator
 *
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "adra_server");
  ros::NodeHandle n;

  int device = 0;
  std::string device_name = ros::this_node::getName() + "/device";
  ros::param::get(device_name.c_str(), device);

  int bus = 0;
  std::string bus_name = ros::this_node::getName() + "/bus";
  ros::param::get(bus_name.c_str(), bus);

  std::string port = "";
  std::string portName = ros::this_node::getName() + "/port";
  ros::param::get(portName.c_str(), port);

  int baud = 0;
  std::string baudName = ros::this_node::getName() + "/baud";
  ros::param::get(baudName.c_str(), baud);

  int id = 0;
  std::string idName = ros::this_node::getName() + "/id";
  ros::param::get(idName.c_str(), id);

  ROS_INFO("[AdraServ] device:%d bus:%d port:%s baud:%d id:%d", device, bus, port.c_str(), baud, id);

  if (baud != 0 && id != 0) {
    const char *ip = port.c_str();
    const char *com = port.c_str();
    if (1 == device || 2 == device)
      adra = new AdraApiSerial(com, baud);
    else if (3 == device)
      adra = new AdraApiTcp((char *)ip, 6001, 0, 1, 5001, baud);
    else if (4 == device)
      adra = new AdraApiUdp((char *)ip, 5001, 0, 1, 6001, baud);

    uint8_t mode;
    int ret = adra->get_motion_mode(id, &mode);
    if (ret == -3) {
      ROS_ERROR("[AdraServ] can not connect the adra.");
      return 0;
    }
  }

  ros::ServiceServer service = n.advertiseService("adra_server", api);
  ROS_INFO("[AdraServ] Ready for adra server.");

  std::string publishName = ros::this_node::getName() + "/publish";
  bool ispublish = false;
  ros::param::get(publishName.c_str(), ispublish);
  ROS_INFO("[AdraServ] ispublish %d", ispublish);
  if (ispublish) {
    chatter_pub = n.advertise<adra_ros::AdraMsg>("adra_publish", 1000);
    std::thread publish;
    publish = std::thread(publishThread, id);
    publish.detach();
    ROS_INFO("[AdraServ] Ready for adra publish.");
  }

  ros::spin();

  return 0;
}
