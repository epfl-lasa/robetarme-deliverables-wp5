/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

/**
 * @brief Mother class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class IRoboticArmBase {
  /* TODO: Implement all the common functions for all the robotic arms, make them pure virtual if the
   function is not implemented in the mother class */
public:
  // TODO: implement all the public members, accessible from everyone owning a class object
  explicit IRoboticArmBase() = default;

protected:
  // TODO: implement all the protected members, accessible from its own and herited classes

private:
  // TODO: implement all the private members, only accessible from its own class
};
