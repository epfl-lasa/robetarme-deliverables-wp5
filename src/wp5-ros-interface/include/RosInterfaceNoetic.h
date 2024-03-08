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
  explicit RosInterfaceNoetic(ros::NodeHandle& nh);

  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> receive_state();
  void send_state(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& data);

private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  std::vector<double> jointsPosition;
  std::vector<double> jointsSpeed;
  std::vector<double> jointsTorque;

  bool init_joint;

  ros::NodeHandle nh_;
  ros::Subscriber sub_state_;
  ros::Publisher pub_state_;
};
