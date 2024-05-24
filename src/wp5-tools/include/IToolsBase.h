/**
 * @file IToolsBase.h
 * @brief Declaration of the IRoboticArmBase class
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @version 0.1
 * @date 2024-03-07
 * @copyright Copyright (c) 2024 - EPFL
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

/**
 * @brief Mother class to create all the prototype functions needed in different robotic arms.
 *
 * This class provides methods to manage a tools with all the necessary functions to control it.
 */
class IToolsBase {
public:
  /**
IToolsBase   */
  IToolsBase() = default;

  /**
   * @brief Destructor for IToolsBase.
   */
  virtual ~IToolsBase() = default;

  /// @brief  get the offset tool
  /// @return  the offset tool
  double getOffset();

protected:
  void takeYaml(std::string name);

private:
  double offsetTool_;
  double offsetTarget_;
  double offsetTotal_;
  std::string taskname_;
};
