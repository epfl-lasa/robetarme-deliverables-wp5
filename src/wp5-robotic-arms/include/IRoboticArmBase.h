/**
 * @file IRoboticArmBase.h
 * @brief Declaration of the IRoboticArmBase class
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @version 0.1
 * @date 2024-03-07
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <robot_model/Model.hpp>
#include <sstream>
#include <string>

// #include <trac_ik/trac_ik.hpp>
#include <vector>

#include "controllers/ControllerFactory.hpp"
#include "state_representation/parameters/ParameterInterface.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"

/**
 * @brief Mother class to create all the prototype functions needed in different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class IRoboticArmBase {
public:
  /**
   * @brief Default constructor for IRoboticArmBase.
   */
  IRoboticArmBase() = default;

  /**
   * @brief Destructor for IRoboticArmBase.
   */
  virtual ~IRoboticArmBase() = default;

  /**
   * @brief Get the name of the robotic arm.
   * @return Name of the robotic arm.
   */
  std::string getName() { return robotName_; }

  /**
   * @brief Get forward kinematics of the robotic arm.
   *
   * @param vectJoint Vector of joint positions.
   * @return Pair representing the Cartesian pose (quaternion and position) of the end effector.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFK(std::vector<double> vectJoint);

  /**
   * @brief Get the inverse kinematics of the robotic arm.
   *
   * @param vectJoint Vector of current joint positions.
   * @param pairQuatPos Pair representing the target Cartesian pose (position and orientation).
   * @return Vector representing the desired joint positions.
   */
  std::vector<double> getIKinematics(std::vector<double> vectJoint,
                                     const std::pair<Eigen::Quaterniond, Eigen::Vector3d> pairQuatPos);

  /**
   * @brief Get the twist (linear and angular velocities) of the robotic arm.
   *
   * @param posJoint Vector of joint positions.
   * @param speedJoint Vector of joint velocities.
   * @return Vector representing the twist (linear and angular velocities) of the end effector.
   */
  Eigen::VectorXd getTwistFromJointState(std::vector<double> posJoint, std::vector<double> speedJoint);

  /**
   * @brief Get the Jacobian matrix of the robotic arm.
   *
   * @param vectJoint Vector of joint positions.
   * @return Matrix representing the Jacobian matrix of the end effector.
   */
  Eigen::MatrixXd getJacobian(std::vector<double> vectJoint);

  /**
   * @brief Get the inverse dynamics of the robotic arm.
   *
   * @param vectJoint Vector of joint positions.
   * @param speedEigen Vector of twist desired.
   * @return Vector representing the joint torques required for the given joint positions and velocities.
   */
  std::vector<double> getInvertVelocities(std::vector<double> vectJoint, Eigen::VectorXd speedEigen);

  /**
   * @brief Low-level controller function for the robotic arm.
   *
   * This method implements the low-level controller functionality for the robotic arm for shotcrete.
   * @param data Tuple containing vectors of joint positions, joint velocities, and joint efforts.
   * @param twist Vector representing the twist for the end effector.
   * @return Vector containing the control commands for the robotic arm.
   */
  virtual std::vector<double> lowLevelController(
      std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& data, Eigen::VectorXd& twist) = 0;

  /**
   * @brief Low-level controller function for the robotic arm with a safety factor.
   *
   * This method implements the low-level controller functionality for the robotic arm for surface finishing.
   * @param data Tuple containing vectors of joint positions, joint velocities, and joint efforts.
   * @param twist Vector representing the twist for the end effector.
   * @param deltaTwist Vector representing the delta twist for the end effector.
   * @return Vector containing the control commands for the robotic arm.
   */
  virtual std::vector<double> lowLevelControllerSF(
      std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& data,
      Eigen::VectorXd& twist,
      Eigen::VectorXd& deltaTwist) = 0;

  /**
   * @brief Original home joint positions of the robotic arm.
   */
  std::vector<double> originalHomeJoint = {};

protected:
  /**
   * @brief Initialization function for inverse kinematics.
   */
  // Protected members
  std::string robotName_ = "";
  std::vector<std::string> jointNames_;
  std::string baseLink_ = "";
  std::string tipLink_ = "";
  std::string tipJoint_ = "";
  std::string referenceFrame_ = "";
  std::string pathUrdf_ = "";
  std::unique_ptr<robot_model::Model> model_;

  std::string paramURDFnJoint_ = "";
  int nJoint_ = 0;
  struct robot_model::InverseKinematicsParameters paramsIK_ = {};
  Eigen::VectorXd transformWrenchToBase(const Eigen::VectorXd& wrench_end_effector,
                                        const Eigen::Vector3d& position_end_effector,
                                        const Eigen::Quaterniond& orientation_end_effector_to_base);

private:
};
