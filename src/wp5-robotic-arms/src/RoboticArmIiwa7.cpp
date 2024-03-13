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
      {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6","iiwa_joint_7"};
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
  
  std::list<std::shared_ptr<ParameterInterface>> parameters;
  parameters.emplace_back(make_shared_parameter("damping", 10.0));
  parameters.emplace_back(make_shared_parameter("stiffness", 5.0));
  parameters.emplace_back(make_shared_parameter("inertia", 1.0));


  command_state = state_representation::JointState(robotName,jointNames);	
  feedback_state = state_representation::JointState(robotName,jointNames);	

  joint_ctrl = make_unique<JointControllerFactory::create_controller>(CONTROLLER_TYPE::IMPEDANCE,parameters, robot);
  // auto joint_ctrl2 = JointControllerFactory::create_controller(CONTROLLER_TYPE::DISSIPATIVE, robot);
  // auto ctrl = CartesianControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE, parameters);
  }


vector<double> RoboticArmIiwa7::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& stateJoints, Eigen::VectorXd& twist) {
  vector<double>& retrievedPosition = get<0>(stateJoints);
  vector<double>& retrievedSpeed = get<1>(stateJoints);
  vector<double>& retrievedTorque = get<2>(stateJoints);
  Eigen::VectorXd positions(retrievedPosition.size());
  for (size_t i = 0; i < retrievedPosition.size(); ++i) {
      positions(i) = retrievedPosition[i];
  }  
  command_state.set_position(positions);
  // auto command_state = JointState::Random("command");
  // auto feedback_state = JointState::Random("feedback");

  // // compute the command output
  // auto command_output = ctrl->compute_command(command_state, feedback_state);


 return retrievedPosition;

}