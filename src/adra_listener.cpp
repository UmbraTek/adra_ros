/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: johnson <johnson@umbratek.com>
 ============================================================================*/
#include "adra_ros/AdraMsg.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const adra_ros::AdraMsg::ConstPtr& msg) {
  ROS_INFO("id: %d position: %f velocity: %f current: %f", msg->id, msg->position, msg->velocity, msg->current);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("adra_publish", 1000, chatterCallback);

  ros::spin();

  return 0;
}
