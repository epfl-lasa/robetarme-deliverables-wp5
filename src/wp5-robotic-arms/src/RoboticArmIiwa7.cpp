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

using namespace std;


RoboticArmIiwa7::RoboticArmIiwa7() {
  pathUrdf = string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/iiwa7.urdf";
  robotName = "iiwa7_robot";
  tipLink = "tool0";
  tipJoint = "wrist_3_joint";
  baseLink = "base";
  jointNames =
      {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  referenceFrame = "base";
  nJoint = 7;
  originalHomeJoint = vector<double>(nJoint, 0.0);
  model = make_unique<robot_model::Model>(robotName, pathUrdf);

  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int max_number_of_iterations = 1000;
  paramsIK = {damp, alpha, gamma, tolerance, max_number_of_iterations};
  }
