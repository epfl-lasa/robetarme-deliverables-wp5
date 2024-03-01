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

#include "../include/RoboticArmUr5.h"

RoboticArmUr5::RoboticArmUr5() {
std::string package_path = ros::package::getPath("wp5-roboti-arms"); 
path_urdf = package_path + "/urdf/ur5.urdf";
robot_name = "ur5_robot";
tipLink  = "tool0";
tipJoint = "wrist_3_joint";
joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
reference_frame = "base";
nJoint    = 6;

model = make_unique<robot_model::Model>(robot_name, path_urdf);

//inverse kinematik with track-ik
baseLink   = "base";
URDF_param = "/ur5/robot_description";
vector_0.assign(nJoint, 0.0);
posJointNext = vector_0;
ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, error, type);  

valid = ikSolver->getKDLChain(chain);
if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
} 

}
