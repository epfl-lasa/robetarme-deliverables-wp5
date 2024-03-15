/**
 * @file RoboticArmUr5.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "IRoboticArmBase.h"

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class RoboticArmUr5 : public IRoboticArmBase {
public:
  // TODO: implement all the public members, accessible from everyone owning a class object
  explicit RoboticArmUr5();
  std::vector<double>
  lowLevelController(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>& stateJoints,
                     Eigen::VectorXd& twist) override;

protected:
  // TODO: implement all the protected members, accessible from its own and herited classes

private:
};
