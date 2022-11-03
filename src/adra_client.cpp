/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include <cstdlib>
#include "adra_ros/Api.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "adra_server_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<adra_ros::Api>("adra_server");
  adra_ros::Api srv;
  srv.request.api_name = "get_motion_mode";
  srv.request.args.clear();
  srv.request.args.push_back("1");
  if (client.call(srv)) {
    int size = srv.response.rets.size();
    ROS_INFO("rets size : %d ,ret[0]: %s ret[1]: %s ret[2]: %s", size, srv.response.rets[0].c_str(),
             srv.response.rets[1].c_str(), srv.response.rets[2].c_str());
  } else {
    ROS_ERROR("Failed to call service ");
    return 1;
  }

  return 0;
}
