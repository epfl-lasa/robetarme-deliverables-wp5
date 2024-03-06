/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief Declaration of the IRoboticArmBase class
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 */

#pragma once

#include <OsqpEigen/OsqpEigen.h>
#include <Utils.h>
#include <cmath>
#include <dynamical_systems/DynamicalSystemFactory.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <pinocchio/fwd.hpp>
#include <robot_model/Model.hpp>
#include <sstream>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>
#include <yaml-cpp/yaml.h>

// #include <trac_ik/trac_ik.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

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
     * @brief Get forward kinematics of the robotic arm.
     *
     * @param vectJoint Vector of joint positions.
     * @return Vector representing the Cartesian pose (position and orientation) of the end effector.
     */
  vector<double> getFK(vector<double> vectJoint);

  // /**
  //  * @brief Get inverse kinematics of the robotic arm.
  //  *
  //  * @param actualJoint Vector of current joint positions.
  //  * @param vectorQuatPos Vector representing the target Cartesian pose (position and orientation).
  //  * @return A pair containing an error code and the vector of next joint positions.
  //  */
  // pair<int, vector<double>> getIK(vector<double> actualJoint, vector<double> vectorQuatPos);

  // /**
  //  * @brief Update the inverse kinematics parameters.
  //  *
  //  * @param err Error threshold for inverse kinematics solver.
  //  * @param timeoutInSecs Timeout value for inverse kinematics solver.
  //  * @param solveTypeStr Solve type for inverse kinematics solver.
  //  */
  // void updateIK(double err, double timeoutInSecs, string solveTypeStr);

  /**
     * @brief Get the twist (linear and angular velocities) of the robotic arm.
     *
     * @param posJoint Vector of joint positions.
     * @param speedJoint Vector of joint velocities.
     * @return Vector representing the twist (linear and angular velocities) of the end effector.
     */
  VectorXd getTwist(vector<double> posJoint, vector<double> speedJoint);

  /**
     * @brief Get the Jacobian matrix of the robotic arm.
     *
     * @param vectJoint Vector of joint positions.
     * @return Matrix representing the Jacobian matrix of the end effector.
     */
  MatrixXd getJacobian(vector<double> vectJoint);

  /**
     * @brief Get the inverse dynamics of the robotic arm.
     *
     * @param vectJoint Vector of joint positions.
     * @param speedEigen Vector of joint velocities.
     * @return Vector representing the joint torques required for the given joint positions and velocities.
     */
  vector<double> getIDynamics(vector<double> vectJoint, VectorXd speedEigen);

  // /**
  //  * @brief Low-level controller function for the robotic arm.
  //  *
  //  * @param data Tuple containing vectors of joint positions, joint velocities, and joint efforts.
  //  */
  // virtual vector<double>  low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& data, vector<double> twist);

protected:
  /**
     * @brief Initialization function for inverse kinematics.
     */
  // Protected members
  std::string robotName = "";
  vector<string> jointNames;
  std::string baseLink = "";
  std::string tipLink = "";
  std::string tipJoint = "";
  std::string referenceFrame = "";
  std::string pathUrdf = "";
  std::unique_ptr<robot_model::Model> model;

  std::string paramURDF = "";
  int nJoint = 0;

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
     * @brief Function for computing speed with quaternion interpolation.
     *
     * @param Pos Vector representing the current position.
     * @param quat2 Vector representing the target quaternion.
     * @param speed Vector representing the target speed.
     * @return Vector representing the computed speed.
     */
  VectorXd speed_func(vector<double> Pos, vector<double> quat2, vector<double> speed);

  // TODO: Add any additional private members as needed
};
