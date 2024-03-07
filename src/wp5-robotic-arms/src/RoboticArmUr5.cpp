/**
 * @file RoboticArmUr5.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "RoboticArmUr5.h"

RoboticArmUr5::RoboticArmUr5() {

pathUrdf = std::string(WP5_ROBOTIC_ARMS_DIR) + "/urdf/ur5.urdf";
robotName = "ur5_robot";
tipLink  = "tool0";
tipJoint = "wrist_3_joint";
baseLink = "base";
jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
referenceFrame = "base";
nJoint    = 6;

model = make_unique<robot_model::Model>(robotName, pathUrdf);

// IRoboticArmBase::initIK();

}

// vector<double>  RoboticArmUr5::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& stateJoints,vector<double> twist) {
//     vector<double>& retrievedPosition = get<0>(stateJoints);

//     Eigen::VectorXd eigenVector = Eigen::Map<Eigen::VectorXd>(twist.data(), twist.size());

//     vector<double> desiredJointSpeed = IRoboticArmBase::getIDynamics(retrievedPosition, eigenVector);
//     return desiredJointSpeed;

// }