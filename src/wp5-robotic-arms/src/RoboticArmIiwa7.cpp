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

using namespace controllers;
using namespace state_representation;
using namespace std;

RoboticArmIiwa7::RoboticArmIiwa7() {
  pathUrdf = string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/iiwa7.urdf";
  robotName = "iiwa7_robot";
  tipLink = "iiwa_link_7";
  tipJoint = "iiwa_joint_7";
  baseLink = "base";
  jointNames =
      {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
  referenceFrame = "ground_plane";
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
  paramsIK = {damp, alpha, gamma, tolerance, max_number_of_iterations};

  parameters.emplace_back(make_shared_parameter("damping", 1.0));
  parameters.emplace_back(make_shared_parameter("stiffness", 10.0));
  parameters.emplace_back(make_shared_parameter("inertia", 1.0));

  command_state = state_representation::JointState(robotName, jointNames);
  feedback_state = state_representation::JointState(robotName, jointNames);

  joint_ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, parameters, *model);
}

vector<double> RoboticArmIiwa7::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& stateJoints,
                                                     Eigen::VectorXd& twist) {
  vector<double>& retrievedPosition = get<0>(stateJoints);
  vector<double>& retrievedSpeed = get<1>(stateJoints);
  vector<double>& retrievedTorque = get<2>(stateJoints);
  Eigen::VectorXd positions = Eigen::Map<Eigen::VectorXd>(retrievedPosition.data(), retrievedPosition.size());
  Eigen::VectorXd velocities = Eigen::Map<Eigen::VectorXd>(retrievedSpeed.data(), retrievedSpeed.size());
  Eigen::VectorXd torques = Eigen::Map<Eigen::VectorXd>(retrievedTorque.data(), retrievedTorque.size());
  feedback_state.set_positions(positions);
  feedback_state.set_velocities(velocities);
  feedback_state.set_torques(torques);

  vector<double> desiredJointSpeed = IRoboticArmBase::getIDynamics(retrievedPosition, twist);
  Eigen::VectorXd JointSpeed = Eigen::Map<Eigen::VectorXd>(desiredJointSpeed.data(), desiredJointSpeed.size());
  command_state.set_velocities(JointSpeed);

  cout << twist << endl;
  // compute the command output
  auto command_output = joint_ctrl->compute_command(command_state, feedback_state);


  Eigen::VectorXd EigentorqueCommand = command_output.get_torques();
  std::vector<double> torqueCommand(EigentorqueCommand.data(), EigentorqueCommand.data() + EigentorqueCommand.size());

  return torqueCommand;
}