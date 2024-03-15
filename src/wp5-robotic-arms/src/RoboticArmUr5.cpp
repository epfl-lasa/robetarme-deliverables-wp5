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
  pathUrdf_ = string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/ur5.urdf";
  robotName_ = "ur5_robot";
  tipLink_ = "tool0";
  tipJoint_ = "wrist_3_joint";
  baseLink_ = "base";
  jointNames_ =
      {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  referenceFrame_ = "base";
  nJoint_ = 6;
  originalHomeJoint = vector<double>(nJoint_, 0.0);
  model_ = make_unique<robot_model::Model>(robotName_, pathUrdf_);
  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int maxNumberOfIterations = 1000;
  paramsIK_ = {damp, alpha, gamma, margin, tolerance, maxNumberOfIterations};
}

vector<double> RoboticArmUr5::lowLevelController(tuple<vector<double>, vector<double>, vector<double>>& stateJoints,
                                                 Eigen::VectorXd& twist) {
  vector<double>& retrievedPosition = get<0>(stateJoints);

  vector<double> desiredJointSpeed = IRoboticArmBase::getInvertVelocities(retrievedPosition, twist);
  return desiredJointSpeed;
}