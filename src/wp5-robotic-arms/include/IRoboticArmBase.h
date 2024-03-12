/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief Declaration of the IRoboticArmBase class
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024
 */

#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <cmath>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <robot_model/Model.hpp>
#include <sstream>

// #include <trac_ik/trac_ik.hpp>
#include <vector>

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

   std::vector<double> getHomeJoint();

  /**
     * @brief Get forward kinematics of the robotic arm.
     *
     * @param vectJoint Vector of joint positions.
     * @return Vector representing the Cartesian pose (position and orientation) of the end effector.
     */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getFK(std::vector<double> vectJoint);

  // /**
  //  * @brief Get inverse kinematics of the robotic arm.
  //  *
  //  * @param actualJoint Vector of current joint positions.
  //  * @param vectorQuatPos Vector representing the target Cartesian pose (position and orientation).
  //  * @return A pair containing an error code and the vector of next joint positions.
  //  */
  std::vector<double> getIKinematics(std::vector<double> vectJoint,
                                     const std::pair<Eigen::Quaterniond, Eigen::Vector3d> pairQuatPos);

  // std::pair<int, std::vector<double>> getIK(std::vector<double> actualJoint, std::vector<double> vectorQuatPos);

  // /**
  //  * @brief Update the inverse kinematics parameters.
  //  *
  //  * @param err Error threshold for inverse kinematics solver.
  //  * @param timeoutInSecs Timeout value for inverse kinematics solver.
  //  * @param solveTypeStr Solve type for inverse kinematics solver.
  //  */
  // void updateIK(double err, double timeoutInSecs, std::string solveTypeStr);

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
  std::vector<double> getIDynamics(std::vector<double> vectJoint, Eigen::VectorXd speedEigen);
  /**
     * @brief Function for computing twist with dynamical system.
     *
     * @param Pos Vector representing the current position.
     * @param quat2 Vector representing the target quaternion.
     * @param speed Vector representing the target speed.
     * @return Vector representing the computed speed.
     */
  Eigen::VectorXd getTwistFromDS(Eigen::Quaterniond quat1, std::pair<Eigen::Quaterniond, Eigen::Vector3d> pairQuatPos);

  // /**
  //  * @brief Low-level controller function for the robotic arm.
  //  *
  //  * @param data Tuple containing vectors of joint positions, joint velocities, and joint efforts.
  //  */
virtual std::vector<double> low_level_controller(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& data,  Eigen::VectorXd& twist) = 0;
std::vector<double> originalHomeJoint =  {};

protected:
  /**
     * @brief Initialization function for inverse kinematics.
     */
  // Protected members
  std::string robotName = "";
  std::vector<std::string> jointNames;
  std::string baseLink = "";
  std::string tipLink = "";
  std::string tipJoint = "";
  std::string referenceFrame = "";
  std::string pathUrdf = "";
  std::unique_ptr<robot_model::Model> model;

  std::string paramURDF = "";
  int nJoint = 0;
  struct robot_model::InverseKinematicsParameters paramsIK = {};

  // ik -----------------

  // int rc = 0;
  // TRAC_IK::TRAC_IK* ikSolver = nullptr;
  // KDL::Chain chain = {};
  // bool valid = false;

  // TRAC_IK::SolveType type = TRAC_IK::Distance;
  // double error = 0.01;
  // double timeoutInSecs = 0.5;
  // void initIK();

private:
  /**
     * @brief Function to interpolating quaternion along the shortest path.
     * @param q1 First quaternion.
     * @param q2 Second quaternion.
     * @param t .
     * @return quaternion
     */
  Eigen::Matrix<double, 4, 1>
  slerpQuaternion(Eigen::Matrix<double, 4, 1>& q1, Eigen::Matrix<double, 4, 1>& q2, double t);

  /**
     * @brief Function fo computing the quaternion product.
     * @param q1 First quaternion.
     * @param q2 Second quaternion.
     * @return quaternion product
     */
  Eigen::Matrix<double, 4, 1> quaternionProduct(Eigen::Matrix<double, 4, 1> q1, Eigen::Matrix<double, 4, 1> q2);
};
