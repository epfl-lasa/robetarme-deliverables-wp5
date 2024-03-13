/**
 * @file RoboticArmIiwa7.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include "IRoboticArmBase.h"
#include "controllers/ControllerFactory.hpp"


/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmIiwa7 : public IRoboticArmBase {
public:
  // TODO: implement all the public members, accessible from everyone owning a class object
  explicit RoboticArmIiwa7();
  std::vector<double> low_level_controller(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& stateJoints,  Eigen::VectorXd& twist) override;

protected:
  // TODO: implement all the protected members, accessible from its own and herited classes
  state_representation::JointState command_state;  
  state_representation::JointState feedback_state;  
  std::unique_ptr<controllers::JointControllerFactory> joint_ctrl;

private:

  // TODO: implement all the private members, only accessible from its own class
};
