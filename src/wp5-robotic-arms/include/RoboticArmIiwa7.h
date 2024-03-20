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

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmIiwa7 : public IRoboticArmBase {
public:
  explicit RoboticArmIiwa7();
  std::vector<double>
  lowLevelController(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& stateJoints,
                     Eigen::VectorXd& twist) override;
  std::vector<double>
  lowLevelControllerSF(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& stateJoints,
                       Eigen::VectorXd& desiredTwist,
                       Eigen::VectorXd& wrenchFromSensor) override;

protected:
  state_representation::CartesianState commandState_;
  state_representation::CartesianState feedbackState_;
  std::shared_ptr<controllers::IController<state_representation::CartesianState>> twistCtrl_;
  std::list<std::shared_ptr<state_representation::ParameterInterface>> parameters_;
};
