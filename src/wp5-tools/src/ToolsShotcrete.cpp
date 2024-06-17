/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "ToolsShotcrete.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>

#include <fstream>

using namespace std;

ToolsShotcrete::ToolsShotcrete(ros::NodeHandle& n) : IToolsBase(n) {
  takeYaml("shotcrete");
  pubCommand_ = nh_.advertise<std_msgs::Bool>("servo_command", 10);
  subState_ = nh_.subscribe("servo_state", 10, &ToolsShotcrete::stateCallback, this);

  activateTool(false);
}
void ToolsShotcrete::stateCallback(const std_msgs::Bool::ConstPtr& msg) { state_ = msg->data; }
bool ToolsShotcrete::getState() { return state_; }
void ToolsShotcrete::activateTool(bool activateTrue) {
  // if True will activate the tool
  // if False will deactivate the tool
  std_msgs::Bool msg;
  msg.data = activateTrue;
  pubCommand_.publish(msg);
}
