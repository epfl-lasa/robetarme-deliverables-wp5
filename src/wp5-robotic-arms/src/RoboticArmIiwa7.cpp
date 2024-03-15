/**
 * @file RoboticArmIiwa7.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */

#include "RoboticArmIiwa7.h"

#include <yaml-cpp/yaml.h>

#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"

using namespace controllers;
using namespace state_representation;
using namespace std;

RoboticArmIiwa7::RoboticArmIiwa7() {
  pathUrdf_ = string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/iiwa7.urdf";
  robotName_ = "iiwa7";
  tipLink_ = "iiwa_link_ee";
  tipJoint_ = "iiwa_joint_ee";
  baseLink_ = "iiwa_link_0";
  jointNames_ = {
      "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
  referenceFrame_ = "iiwa_link_0";
  nJoint_ = 7;
  originalHomeJoint = vector<double>(nJoint_, 0.0);
  model_ = make_unique<robot_model::Model>(robotName_, pathUrdf_);
  auto robot = robot_model::Model(robotName_, pathUrdf_);

  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int maxNumberOfIterations = 1000;
  paramsIK_ = {damp, alpha, gamma, margin, tolerance, maxNumberOfIterations};
  Eigen::ArrayXd torques(nJoint_);

  commandState_ = state_representation::CartesianState(robotName_, referenceFrame_);
  feedbackState_ = state_representation::CartesianState(robotName_, referenceFrame_);

  string yamlPath = string(WP5_ROBOTIC_ARMS_DIR) + "/config/robot_config.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);

  YAML::Node robotNode = config["Iiwa7"];

  double linearPrincipledamping = robotNode["linear_principle_damping"].as<double>();
  double linearOrthogonalDamping = robotNode["linear_orthogonal_damping"].as<double>();
  double angularStiffness = robotNode["angular_stiffness"].as<double>();
  double angularDamping = robotNode["angular_damping"].as<double>();

  parameters_.emplace_back(make_shared_parameter("linear_principle_damping", linearPrincipledamping));
  parameters_.emplace_back(make_shared_parameter("linear_orthogonal_damping", linearOrthogonalDamping));
  parameters_.emplace_back(make_shared_parameter("angular_stiffness", angularStiffness));
  parameters_.emplace_back(make_shared_parameter("angular_damping", angularDamping));

  twistCtrl_ = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::COMPLIANT_TWIST, parameters_, *model_);
}

vector<double> RoboticArmIiwa7::lowLevelController(tuple<vector<double>, vector<double>, vector<double>>& stateJoints,
                                                   Eigen::VectorXd& desiredTwist) {
  //set_up the feeedback_State
  vector<double>& retrievedPosition = get<0>(stateJoints);
  vector<double>& retrievedSpeed = get<1>(stateJoints);
  vector<double>& retrievedTorque = get<2>(stateJoints);

  Eigen::VectorXd positions = Eigen::Map<Eigen::VectorXd>(retrievedPosition.data(), retrievedPosition.size());
  Eigen::VectorXd velocities = Eigen::Map<Eigen::VectorXd>(retrievedSpeed.data(), retrievedSpeed.size());
  Eigen::VectorXd torques = Eigen::Map<Eigen::VectorXd>(retrievedTorque.data(), retrievedTorque.size());

  Eigen::VectorXd actualTwist = getTwistFromJointState(retrievedPosition, retrievedSpeed);
  feedbackState_.set_twist(actualTwist);
  commandState_.set_twist(desiredTwist);

  state_representation::JointPositions JointPositions;

  JointPositions = state_representation::JointPositions(robotName_, jointNames_, positions);

  state_representation::Jacobian jacobianObject = model_->compute_jacobian(JointPositions);

  // compute the command output
  auto commandOutput = twistCtrl_->compute_command(commandState_, feedbackState_);
  Eigen::VectorXd wrench = commandOutput.get_wrench();
  Eigen::VectorXd EigentorqueCommand = jacobianObject.transpose() * wrench;

  vector<double> torqueCommand(EigentorqueCommand.data(), EigentorqueCommand.data() + EigentorqueCommand.size());
  return torqueCommand;
}