/**
 * @file RosInterfaceNoetic.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "ros/ros.h"
#include <ros/package.h>
#include <cstdlib>
#include <yaml-cpp/yaml.h>  
#include <vector>
#include <tuple>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

class RosInterfaceNoetic {
  public:

    explicit RosInterfaceNoetic(ros::NodeHandle& nh) ;
    tuple<vector<double>, vector<double>, vector<double>> receive_state() ;
    void send_state(tuple<vector<double>, vector<double>, vector<double>>& data) ;

  private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    vector<double> jointsPosition;
    vector<double> jointsSpeed;
    vector<double> jointsTorque;
    bool init_joint;

    ros::NodeHandle nh_; // Member field to store the NodeHandle
    ros::Subscriber sub_state_;
    ros::Publisher pub_state_;
};
