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
#include "adra_ros/AdraMsg.h"
#include "adra_ros/Api.h"
#include "ros/ros.h"

static AdraApiBase *adra = NULL;
static ros::Publisher chatter_pub;

void check_ret(int ret, const char *str) {
  if (ret == 0)
    printf("Good! successfully %s\n", str);
  else
    printf("Error! Failed %s %d\n", str, ret);
}
constexpr unsigned int hash(const char *s, int off = 0) { return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off]; }

void connect(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
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
  ROS_INFO("device=%s  baud=%ld", req.args[0].c_str(), baud);

  adra = new AdraApiSerial(req.args[0].c_str(), baud);

  // if (!socket_adra_->is_ok()) {
  //   res.rets.push_back("-1");
  //   char str[50];
  //   sprintf(str, "[UtManage] Error: socket_file open failed, %s\n", req.args[0].c_str());
  //   res.rets.push_back(str);
  //   return;
  // }

  uint8_t mode;
  int ret = adra->get_motion_mode(id, &mode);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("can not connect the adra");
    return;
  } else {
    res.rets.push_back("0");
    res.rets.push_back("connect seccess");
  }
}

void disconnect(adra_ros::Api::Response &res) {
  if (adra != NULL) {
    delete adra;
    adra = NULL;
    res.rets.push_back("0");
    res.rets.push_back("disconnect seccess");
    return;
  }
  res.rets.push_back("-1");
  res.rets.push_back("server have not connect adra");
}
bool check_connect(adra_ros::Api::Response &res) {
  if (adra == NULL) {
    res.rets.push_back("-1");
    res.rets.push_back("server have not connect adra");
    return false;
  } else {
    return true;
  }
}
bool check_arg_count(adra_ros::Api::Request &req, adra_ros::Api::Response &res, int count) {
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
int get_id(adra_ros::Api::Request &req) {
  int id;
  // ROS_INFO("id:%s",req.args[0].c_str());
  id = std::stoi(req.args[0].c_str());
  return id;
}
void into_motion_mode_pos(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  int ret = adra->into_motion_mode_pos(id);
  res.rets.push_back(std::to_string(ret));
}
void into_motion_mode_vel(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  int ret = adra->into_motion_mode_vel(id);
  res.rets.push_back(std::to_string(ret));
}
void into_motion_mode_tau(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  int ret = adra->into_motion_mode_tau(id);
  res.rets.push_back(std::to_string(ret));
}
void get_motion_mode(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  uint8_t mode = -1;
  int ret = adra->get_motion_mode(id, &mode);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("can not connect the adra");
    return;
  }
  switch (mode) {
    case 1:
      res.rets.push_back(std::to_string(ret));
      res.rets.push_back("current is position mode");
      return;
    case 2:
      res.rets.push_back(std::to_string(ret));
      res.rets.push_back("current is velicity mode");
      return;
    case 3:
      res.rets.push_back(std::to_string(ret));
      res.rets.push_back("current is current mode");
      return;
  }
}
void into_motion_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  int ret = adra->into_motion_enable(id);
  res.rets.push_back(std::to_string(ret));
}
void into_motion_disable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  int ret = adra->into_motion_disable(id);
  res.rets.push_back(std::to_string(ret));
}
void get_motion_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  uint8_t mode = -1;
  int ret = adra->get_motion_enable(id, &mode);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("can not connect the adra");
    return;
  }
  switch (mode) {
    case 0:
      res.rets.push_back(std::to_string(ret));
      res.rets.push_back("motion is disable");
      return;
    case 1:
      res.rets.push_back(std::to_string(ret));
      res.rets.push_back("motion is enable");
      return;
  }
}
void into_brake_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  int ret = adra->into_brake_enable(id);
  res.rets.push_back(std::to_string(ret));
}
void into_brake_disable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  int ret = adra->into_brake_disable(id);
  res.rets.push_back(std::to_string(ret));
}
void get_brake_enable(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  uint8_t mode = -1;
  int ret = adra->get_brake_enable(id, &mode);
  if (ret == -3) {
    res.rets.push_back("-3");
    res.rets.push_back("can not connect the adra");
    return;
  }
  switch (mode) {
    case 0:
      res.rets.push_back(std::to_string(ret));
      res.rets.push_back("brake is disable");
      return;
    case 1:
      res.rets.push_back(std::to_string(ret));
      res.rets.push_back("brake is enable");
      return;
  }
}
void get_pos_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  float pos = -1;
  int ret = adra->get_pos_target(id, &pos);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(pos));
}
void set_pos_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 2) == false) return;
  int id = get_id(req);
  std::string::size_type sz;
  float pos = std::stof(req.args[1].c_str());
  ROS_INFO("set_pos_target : %s %f", req.args[1].c_str(), pos);
  int ret = adra->set_pos_target(id, pos);
  res.rets.push_back(std::to_string(ret));
}
void get_pos_current(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  float pos = -1;
  int ret = adra->get_pos_current(id, &pos);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(pos));
}
void get_vel_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  float vel = -1;
  int ret = adra->get_vel_target(id, &vel);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(vel));
}
void set_vel_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 2) == false) return;
  int id = get_id(req);
  std::string::size_type sz;
  float vel = std::stof(req.args[1].c_str());
  ROS_INFO("set_vel_target : %s %f", req.args[1].c_str(), vel);
  int ret = adra->set_vel_target(id, vel);
  res.rets.push_back(std::to_string(ret));
}
void get_vel_current(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  float vel = -1;
  int ret = adra->get_vel_current(id, &vel);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(vel));
}
void get_tau_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  float tau = -1;
  int ret = adra->get_tau_target(id, &tau);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(tau));
}
void set_tau_target(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 2) == false) return;
  int id = get_id(req);
  std::string::size_type sz;
  float tau = std::stof(req.args[1].c_str());
  ROS_INFO("set_tau_target : %s %f", req.args[1].c_str(), tau);
  int ret = adra->set_tau_target(id, tau);
  res.rets.push_back(std::to_string(ret));
}
void get_tau_current(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  if (check_connect(res) == false) return;
  if (check_arg_count(req, res, 1) == false) return;
  int id = get_id(req);
  float tau = -1;
  int ret = adra->get_tau_current(id, &tau);
  res.rets.push_back(std::to_string(ret));
  res.rets.push_back(std::to_string(tau));
}
bool api(adra_ros::Api::Request &req, adra_ros::Api::Response &res) {
  ROS_INFO("request: api_name=%s", req.api_name.c_str());
  switch (hash(req.api_name.c_str())) {
    case hash("connect"):
      connect(req, res);
      break;
    case hash("disconnect"):
      disconnect(res);
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
    case hash("get_pos_target"):
      get_pos_target(req, res);
      break;
    case hash("set_pos_target"):
      set_pos_target(req, res);
      break;
    case hash("get_pos_current"):
      get_pos_current(req, res);
      break;
    case hash("get_vel_target"):
      get_vel_target(req, res);
      break;
    case hash("set_vel_target"):
      set_vel_target(req, res);
      break;
    case hash("get_vel_current"):
      get_vel_current(req, res);
      break;
    case hash("get_tau_target"):
      get_tau_target(req, res);
      break;
    case hash("set_tau_target"):
      set_tau_target(req, res);
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
    float pos = -1;
    int ret = adra->get_pos_current(id, &pos);
    msg.position = pos;
    float vel = -1;
    ret = adra->get_vel_current(id, &vel);
    msg.velocity = vel;
    float tau = -1;
    ret = adra->get_tau_current(id, &tau);
    msg.current = tau;

    chatter_pub.publish(msg);
    usleep(100);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adra_server");
  ros::NodeHandle n;

  std::string portName = ros::this_node::getName() + "/port";
  std::string port = "";
  ros::param::get(portName.c_str(), port);
  std::string baudName = ros::this_node::getName() + "/baud";
  int baud = 0;
  ros::param::get(baudName.c_str(), baud);
  std::string idName = ros::this_node::getName() + "/id";
  int id = 0;
  ros::param::get(idName.c_str(), id);
  ROS_INFO("port: %s baud: %d id : %d", port.c_str(), baud, id);
  if (baud != 0 && id != 0) {
    adra = new AdraApiSerial(port.c_str(), baud);
    uint8_t mode;
    int ret = adra->get_motion_mode(id, &mode);
    if (ret == -3) {
      ROS_ERROR("can not connect the adra");
      return -1;
    }
  }

  ros::ServiceServer service = n.advertiseService("adra_server", api);

  ROS_INFO("Ready for adra server.");

  std::string publishName = ros::this_node::getName() + "/publish";
  bool ispublish = false;
  ros::param::get(publishName.c_str(), ispublish);
  ROS_INFO("ispublish %d", ispublish);
  if (ispublish) {
    chatter_pub = n.advertise<adra_ros::AdraMsg>("adra_publish", 1000);
    std::thread publish;
    publish = std::thread(publishThread, id);
    publish.detach();
    ROS_INFO("Ready for adra publish.");
  }

  ros::spin();

  return 0;
}
