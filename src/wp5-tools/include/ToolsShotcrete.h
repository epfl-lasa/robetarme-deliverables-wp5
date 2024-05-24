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

#include "IToolsBase.h"
#include <functional>

/**
 * @brief Child class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class ToolsShotcrete : public IToolsBase {
public:
  explicit ToolsShotcrete();
  // void activateTool(std::function<void()> func);

protected:
private:
  double offsetTool_;
  double offsetTarget_;
};
