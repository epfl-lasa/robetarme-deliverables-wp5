/**
 * @file ToolsShotcrete.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#pragma once

#include <std_msgs/Bool.h>

#include <functional>

#include "IToolsBase.h"
/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class ToolsShotcrete : public IToolsBase {
public:
  explicit ToolsShotcrete(ros::NodeHandle& n);
  void activateTool(bool flag) override;

protected:
private:
  double offsetTool_;
  double offsetTarget_;
  ros::Publisher pubCommand_; /**< Publisher for robot state. */
};
