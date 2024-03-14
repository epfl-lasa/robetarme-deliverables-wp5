/**
 * @file RosInterfaceNoetic.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief Create a ROS interface with respect to the ROS version to communicate with the robotic arm
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RosInterfaceNoetic.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <yaml-cpp/yaml.h>

using namespace std;

RosInterfaceNoetic::RosInterfaceNoetic(ros::NodeHandle& nh, string robot) : nh_(nh), robotName(robot) {
  // Load parameters from YAML file
  try {
    std::string package_path = ros::package::getPath("wp5-ros-interface");

    // Load parameters from YAML file
    std::string yaml_path = package_path + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Print information about robotname field
    YAML::Node robotNode = config[robotName];

    // Attempt to access the "njoint" field within the robot
    nJoint = robotNode["njoint"].as<int>();

    string actualStateTopic = robotNode["joint_topic"].as<string>();
    string commandStateTopic = robotNode["joint_command"].as<string>();

    // Initialization
    jointsPosition.assign(nJoint, 0.0);
    jointsSpeed.assign(nJoint, 0.0);
    jointsTorque.assign(nJoint, 0.0);
    init_joint = false;

    // ROS init
    sub_state_ = nh_.subscribe(actualStateTopic, 10, &RosInterfaceNoetic::jointStateCallback, this);
    pub_state_ = nh_.advertise<std_msgs::Float64MultiArray>(commandStateTopic, 1000);
  }
  catch (const YAML::Exception& e) {
    ROS_ERROR_STREAM("Error loading YAML file: " << e.what());
  }

  // Wait for ROS master to be connected
  while (!ros::master::check()) {
    ROS_INFO("Waiting for ROS master to be connected...");
    ros::Duration(1.0).sleep(); // Sleep for 1 second before checking again
  }

  // Wait for the callback to be called at least once
  while (!init_joint) {
    ROS_INFO("Waiting for the callback to be called...");
    ros::Duration(1.0).sleep(); // Sleep for 1 second before checking again
    ros::spinOnce();            // Ensure the callback is called
  }
}

void RosInterfaceNoetic::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (!msg->position.empty()) {
    jointsPosition = msg->position; // Update the position vector
    jointsSpeed = msg->velocity;    // Update the speed vector
    jointsTorque = msg->effort;     // Update the torque vector

    if (robotName == "Ur5") {
      // swap the position to have each joint in the kinematic order, ONLY FOR UR%
      swap(jointsPosition[0], jointsPosition[2]);
      swap(jointsSpeed[0], jointsSpeed[2]);
    }
    init_joint = true;

  } else {
    ROS_WARN("Received joint positions are empty.");
  }
}

tuple<vector<double>, vector<double>, vector<double>> RosInterfaceNoetic::receive_state() {
  ros::spinOnce();

  // Create a tuple and fill it with existing vectors
  tuple<vector<double>, vector<double>, vector<double>> stateJoints =
      make_tuple(jointsPosition, jointsSpeed, jointsTorque);

  return stateJoints;
}

void RosInterfaceNoetic::send_state(vector<double>& data) {

  std_msgs::Float64MultiArray nextJointMsg;
  nextJointMsg.data = data;
  pub_state_.publish(nextJointMsg);
}