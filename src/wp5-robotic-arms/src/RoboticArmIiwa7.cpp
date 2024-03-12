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
  tipLink = "iiwa_link_7";
  tipJoint = "iiwa_joint_7";
  baseLink = "base";
  jointNames =
      {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6","iiwa_joint_7"};
  referenceFrame = "ground_plane";
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


vector<double> RoboticArmIiwa7::low_level_controller(tuple<vector<double>, vector<double>, vector<double>>& stateJoints, Eigen::VectorXd& twist) {
    vector<double>& retrievedPosition = get<0>(stateJoints);


    Eigen::MatrixXd temp_stiffness = gain_scale * this->stiffness;
    Eigen::MatrixXd temp_damping = gain_scale * this->damping;
    this->controller_->set_parameter_value("stiffness", temp_stiffness);
    this->controller_->set_parameter_value("damping", temp_damping);

    // RCLCPP_INFO(this->get_logger(), "Twist command : [%.2f, %.2f, %.2f] \n", twist_command.data()(0), twist_command.data()(1), twist_command.data()(2));
    // RCLCPP_INFO(this->get_logger(), "Gain weights : [%.2f, %.2f] \n", gain_weights.at(0),  gain_weights.at(1));
    // RCLCPP_INFO(this->get_logger(), "Stiffness : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", temp_stiffness(0,0), temp_stiffness(1,1),temp_stiffness(2,2),
    //   temp_stiffness(3,3),temp_stiffness(4,4),temp_stiffness(5,5),temp_stiffness(6,6));
    // RCLCPP_INFO(this->get_logger(), "Damping : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", temp_damping(0,0), temp_damping(1,1),temp_damping(2,2),
    //   temp_damping(3,3),temp_damping(4,4),temp_damping(5,5),temp_damping(6,6));

    // Impedance controller version 2023
    this->command_message_.joint_state = this->controller_->compute_command(
        q_dot_desired, this->state_message_.joint_state);

    // Clamp torques
    this->command_message_.joint_state.clamp_state_variable(20.0, state_representation::JointStateVariable::TORQUES);
    return desiredJointSpeed;

}