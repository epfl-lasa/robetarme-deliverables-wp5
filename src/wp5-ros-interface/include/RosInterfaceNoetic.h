/**
 * @file RosInterfaceNoetic.h
 * @brief Create a ROS interface with respect to the ROS version to communicate with the robotic arm
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @version 0.1
 * @date 2024-03-07
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once

#include <geometry_msgs/WrenchStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cstdlib>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <tuple>
#include <vector>

/**
 * @brief Class for ROS interface compatible with ROS Noetic
 */
class RosInterfaceNoetic {
public:
  /**
   * @brief Constructor.
   * @param n Node handle for ROS.
   * @param robotName Name of the robot.
   */
  explicit RosInterfaceNoetic(ros::NodeHandle& n, std::string robotName);

  /**
   * @brief Receives the state of the robot.
   * @return Tuple containing joint positions, velocities, and torques.
   */
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> receiveState();

  /**
   * @brief Sends the state of the robot.
   * @param data Vector containing the state data.
   */
  void sendState(std::vector<double>& data);

  /**
   * @brief Sets the Cartesian twist of the end-effector.
   * @param data Vector containing the twist data.
   */
  void setCartesianTwist(std::vector<double>& data);

  /**
   * @brief Sets the desired twist of the dynamical system.
   * @param data Vector containing the twist data.
   */
  void setDesiredDsTwist(std::vector<double>& data);

  /**
   * @brief Sets the actual Pose of the end-effector.
   * @param data Vector containing the quaternion (x,y,z,w) and position (x,y,z) data.
   */
  void setCartesianPose(std::pair<Eigen::Quaterniond, Eigen::Vector3d> pairActualQuatPos);

  /**
   * @brief Receives wrench data from force/torque sensor.
   * @return Vector containing wrench data.
   */
  std::vector<double> receiveWrench();

  /**
   * @brief set a bool to activate or desactivate the spray.
   * @param data bool to activate (true) or disactivate the spray
   */
  void setActuatorSpray(bool data);

private:
  /**
   * @brief Callback function for joint state subscriber.
   * @param msg Joint state message.
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * @brief Callback function for force/torque sensor subscriber.
   * @param msg WrenchStamped message.
   */
  void FTCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  std::vector<double> jointsPosition_; /**< Vector containing joint positions. */
  std::vector<double> jointsSpeed_;    /**< Vector containing joint speeds. */
  std::vector<double> jointsTorque_;   /**< Vector containing joint torques. */
  std::vector<double> wrenchSensor_;   /**< Vector containing wrench data from sensor. */
  std::string robotName_;              /**< Name of the robot. */
  std::deque<std::vector<double>> wrench_buffer_;
  size_t buffer_size_ = 10; // Size of the buffer for mean filtering

  std::vector<double> computeMean();
  void addWrenchData(const std::vector<double>& new_data);
  void computeMeanWrench();

  bool initJoint_;                           /**< Flag indicating if joint state is initialized. */
  bool initFTsensor_;                        /**< Flag indicating if force/torque sensor is initialized. */
  int nJoint_;                               /**< Number of joints. */
  ros::NodeHandle nh_;                       /**< Node handle for ROS. */
  ros::Subscriber subFTsensor_;              /**< Subscriber for force/torque sensor. */
  ros::Subscriber subState_;                 /**< Subscriber for joint state. */
  ros::Publisher pubState_;                  /**< Publisher for robot state. */
  ros::Publisher pubStateDS_;                /**< Publisher for dynamical system state. */
  ros::Publisher pubActuatorSpray_;          /**< Publisher for actuator for the spray. */
  ros::Publisher pubStateCartesianTwistEEF_; /**< Publisher for Cartesian twist of end-effector. */
  ros::Publisher pubStateCartesianPoseEEF_;  /**< Publisher for Cartesian pose of end-effector. */
};
