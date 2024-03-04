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
pathUrdf = "../urdf/ur5.urdf";
robotName = "ur5_robot";
tipLink  = "tool0";
tipJoint = "wrist_3_joint";
jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
referenceFrame = "base";
nJoint    = 6;

model = make_unique<robot_model::Model>(robotName, pathUrdf);

//inverse kinematik with track-ik
baseLink   = "base";
paramURDF = "/ur5/robot_description";
ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, paramURDF, timeoutInSecs, error, type);  

valid = ikSolver->getKDLChain(chain);
if (!valid) {
    cout << "There was no valid KDL chain found"<< endl;
} 

}

// void RoboticArmUr5::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& data) {
//     // Implement the low-level controller logic specific to RoboticArmUr5
//     // ...
// }