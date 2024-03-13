/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "RoboticArmUr5.h"
#include "controllers/ControllerFactory.hpp"


using namespace controllers;
using namespace state_representation;
using namespace std;

RoboticArmUr5::RoboticArmUr5() {
  pathUrdf = string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/ur5.urdf";
  robotName = "ur5_robot";
  tipLink = "tool0";
  tipJoint = "wrist_3_joint";
  baseLink = "base";
  jointNames =
      {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  referenceFrame = "base";
  nJoint = 6;
  originalHomeJoint = vector<double>(nJoint, 0.0);
  model = make_unique<robot_model::Model>(robotName, pathUrdf);

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

  std::shared_ptr<controllers::IController<state_representation::JointState>> joint_ctrl
  joint_ctrl = JointControllerFactory::create_controller(CONTROLLER_TYPE::IMPEDANCE,parameters);
  std::cout << "Type of ctrl: " << typeid(joint_ctrl).name() << std::endl;
}

vector<double> RoboticArmUr5::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& stateJoints,
                                                   Eigen::VectorXd& twist) {
  vector<double>& retrievedPosition = get<0>(stateJoints);

  vector<double> desiredJointSpeed = IRoboticArmBase::getIDynamics(retrievedPosition, twist);
  return desiredJointSpeed;
}