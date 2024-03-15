/**
 * @file RosInterfaceNoetic.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief Create a ROS interface with respect to the ROS version to communicate with the robotic arm
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <cstdlib>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tuple>
#include <vector>

class RosInterfaceNoetic {
public:
  explicit RosInterfaceNoetic(ros::NodeHandle& n, std::string robotName);

  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> receiveState();
  void sendState(std::vector<double>& data);

private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  std::vector<double> jointsPosition_;
  std::vector<double> jointsSpeed_;
  std::vector<double> jointsTorque_;
  std::string robotName_;

  bool initJoint_;
  int nJoint_;
  ros::NodeHandle nh_;
  ros::Subscriber subState_;
  ros::Publisher pubState_;
};
