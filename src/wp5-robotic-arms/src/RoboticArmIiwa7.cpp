/**
 * @file RoboticArmIiwa7.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "RoboticArmIiwa7.h"
#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include <yaml-cpp/yaml.h>

using namespace controllers;
using namespace state_representation;
using namespace std;

RoboticArmIiwa7::RoboticArmIiwa7() {
  pathUrdf = string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/iiwa7.urdf";
  robotName = "iiwa7";
  tipLink = "iiwa_link_ee";
  tipJoint = "iiwa_joint_ee";
  baseLink = "iiwa_link_0";
  jointNames =
      {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
  referenceFrame = "iiwa_link_0";
  nJoint = 7;
  originalHomeJoint = vector<double>(nJoint, 0.0);
  model = make_unique<robot_model::Model>(robotName, pathUrdf);
  auto robot = robot_model::Model(robotName, pathUrdf);

  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int max_number_of_iterations = 1000;
  paramsIK = {damp, alpha, gamma, margin, tolerance, max_number_of_iterations};
  Eigen::ArrayXd torques(nJoint);

  torques << 120, 120, 80, 80, 80, 30, 30;
  maxTorquesJoints = torques;

  // command_state = state_representation::JointState(robotName, jointNames);
  // feedback_state = state_representation::JointState(robotName, jointNames);
  command_state = state_representation::CartesianState(robotName, referenceFrame);
  feedback_state = state_representation::CartesianState(robotName, referenceFrame);
  JointState = state_representation::JointState(robotName, jointNames);

  parameters.emplace_back(make_shared_parameter("linear_principle_damping", 500.0));
  parameters.emplace_back(make_shared_parameter("linear_orthogonal_damping", 100.0));
  parameters.emplace_back(make_shared_parameter("angular_stiffness", 100.0));
  parameters.emplace_back(make_shared_parameter("angular_damping", 100.0));

  twist_ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::COMPLIANT_TWIST, parameters, *model);

  // joint_ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::VELOCITY_IMPEDANCE, *model);
  // string yaml_path = string(WP5_ROBOTIC_ARMS_DIR) + "/config/robot_config.yaml";
  // YAML::Node config = YAML::LoadFile(yaml_path);

  // YAML::Node robotNode = config["Iiwa7"];

  // // Extract stiffness values from the YAML node
  // Eigen::VectorXd diagonalStiffness(7);
  // for (int i = 0; i < nJoint; ++i) {
  //   diagonalStiffness(i) = robotNode["stiffness"][i].as<double>();
  // }
  // // Create a diagonal matrix using the extracted values
  // Eigen::MatrixXd stiffness = diagonalStiffness.asDiagonal();
  // joint_ctrl->set_parameter_value("stiffness", stiffness);

  // // Extract stiffness values from the YAML node
  // Eigen::VectorXd diagonalDamping(7);
  // for (int i = 0; i < nJoint; ++i) {
  //   diagonalDamping(i) = robotNode["damping"][i].as<double>();
  // }
  // // Create a diagonal matrix using the extracted values
  // Eigen::MatrixXd damping = diagonalDamping.asDiagonal();
  // joint_ctrl->set_parameter_value("damping", damping);

  // Eigen::MatrixXd inertia = Eigen::MatrixXd::Zero(7, 7);
  // inertia.diagonal() << 0, 0, 0, 0, 0, 0, 0;
  // joint_ctrl->set_parameter_value("inertia", inertia);
}

vector<double> RoboticArmIiwa7::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& stateJoints,
                                                     Eigen::VectorXd& desiredTwist) {
  //set_up the feeedback_State
  vector<double>& retrievedPosition = get<0>(stateJoints);
  vector<double>& retrievedSpeed = get<1>(stateJoints);
  vector<double>& retrievedTorque = get<2>(stateJoints);

  Eigen::VectorXd positions = Eigen::Map<Eigen::VectorXd>(retrievedPosition.data(), retrievedPosition.size());
  Eigen::VectorXd velocities = Eigen::Map<Eigen::VectorXd>(retrievedSpeed.data(), retrievedSpeed.size());
  Eigen::VectorXd torques = Eigen::Map<Eigen::VectorXd>(retrievedTorque.data(), retrievedTorque.size());

  Eigen::VectorXd actualTwist = getTwistFromJointState(retrievedPosition, retrievedSpeed);
  feedback_state.set_twist(actualTwist);
  command_state.set_twist(desiredTwist);

  // JointState.set_positions(positions);
  // JointState.set_velocities(velocities);
  // JointState.set_torques(torques);

  state_representation::JointPositions JointPositions;

  JointPositions = state_representation::JointPositions(robotName, jointNames, positions);

  state_representation::Jacobian jacobianObject = model->compute_jacobian(JointPositions);

  // cout << "jacobian:" << jacobianObject.data() << endl;

  // compute the command output
  // auto command_output = twist_ctrl->compute_command(command_state, feedback_state, jacobianObject);
  auto command_output = twist_ctrl->compute_command(command_state, feedback_state);
  // clamp max force
  // Eigen::ArrayXd NoiseRatio(nJoint);
  // command_output.clamp_state_variable(maxTorquesJoints, JointStateVariable::TORQUES, NoiseRatio);
  Eigen::VectorXd wrench = command_output.get_wrench();
  Eigen::VectorXd EigentorqueCommand = jacobianObject.transpose() * wrench;

  vector<double> torqueCommand(EigentorqueCommand.data(), EigentorqueCommand.data() + EigentorqueCommand.size());
  return torqueCommand;
}
// vector<double> RoboticArmIiwa7::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& stateJoints,
//                                                      Eigen::VectorXd& twist) {
//   //set_up the feeedback_State
//   vector<double>& retrievedPosition = get<0>(stateJoints);
//   vector<double>& retrievedSpeed = get<1>(stateJoints);
//   vector<double>& retrievedTorque = get<2>(stateJoints);
//   Eigen::VectorXd positions = Eigen::Map<Eigen::VectorXd>(retrievedPosition.data(), retrievedPosition.size());
//   Eigen::VectorXd velocities = Eigen::Map<Eigen::VectorXd>(retrievedSpeed.data(), retrievedSpeed.size());
//   Eigen::VectorXd torques = Eigen::Map<Eigen::VectorXd>(retrievedTorque.data(), retrievedTorque.size());
//   feedback_state.set_positions(positions);
//   feedback_state.set_velocities(velocities);
//   feedback_state.set_torques(torques);

//   // compute qdot
//   vector<double> desiredJointSpeed = IRoboticArmBase::getIDynamics(retrievedPosition, twist);
//   Eigen::VectorXd JointSpeed = Eigen::Map<Eigen::VectorXd>(desiredJointSpeed.data(), desiredJointSpeed.size());
//   command_state.set_velocities(JointSpeed);

//   // compute the command output
//   auto command_output = joint_ctrl->compute_command(command_state, feedback_state);

//   // clamp max force
//   Eigen::ArrayXd NoiseRatio(nJoint);
//   command_output.clamp_state_variable(maxTorquesJoints, JointStateVariable::TORQUES, NoiseRatio);

//   Eigen::VectorXd EigentorqueCommand = command_output.get_torques();
//   vector<double> torqueCommand(EigentorqueCommand.data(), EigentorqueCommand.data() + EigentorqueCommand.size());
//   return torqueCommand;
// }