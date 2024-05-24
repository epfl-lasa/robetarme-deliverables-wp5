#pragma once

/**
 * @file
 * @brief This file contains the declaration of the ITaskBase class and its associated enums and dependencies.
 */

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "IToolsBase.h"
#include "PathPlanner.h"
#include "RoboticArmIiwa7.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "ToolsShotcrete.h"
#include "ToolsSurfaceFinishing.h"
#include "visualization_msgs/Marker.h"

/**
 * @brief Enum representing different types of tasks.
 */
enum TaskType : int8_t {
  TASK_UNDEFINED = -1, /**< Undefined task type. */
  SHOTCRETE,           /**< Shotcrete task type. */
  SURFACE_FINISHING,   /**< Surface finishing task type. */
  SAND_BLASTING,       /**< Sand blasting task type. */
  MAM_REBARS,          /**< MAM rebar task type. */
  NB_TASKS             /**< Number of task types. Keep at the end of enum. */
};

/**
 * @brief Base class for tasks.
 */
class ITaskBase {
public:
  /**
   * @brief Map of task names to TaskType enums.
   */
  inline static const std::map<std::string, TaskType> taskTypes{{"shotcrete", SHOTCRETE},
                                                                {"surface_finishing", SURFACE_FINISHING},
                                                                {"sand_blasting", SAND_BLASTING},
                                                                {"mam_rebars", MAM_REBARS}};

  /**
   * @brief Flag indicating if initialization has been checked.
   */
  bool checkInitialization = false;

  /**
   * @brief Flag indicating if task completion has been checked.
   */
  bool checkFinish = false;

  /**
   * @brief Flag indicating if path computation has been checked.
   */
  bool checkPath = false;

  /**
   * @brief Flag indicating if homing position has been checked.
   */
  bool checkHomingPosition = false;

  /**
   * @brief Flag indicating if working position has been checked.
   */
  bool checkWorkingPosition = false;

  /**
   * @brief Constructor.
   * @param nh Node handle for ROS.
   * @param freq Frequency for ROS loop.
   * @param robotName Name of the robot.
   */
  ITaskBase(ros::NodeHandle& nh, double freq, std::string robotName);

  /**
   * @brief Initializes the task.
   * @return True if initialization is successful, false otherwise.
   */
  bool initialize();

  /**
   * @brief Computes the path for the task.
   * @return True if path computation is successful, false otherwise.
   */
  virtual bool computePath() = 0;

  /**
   * @brief Executes the task.
   * @return True if execution is successful, false otherwise.
   */
  virtual bool execute() = 0;

  /**
   * @brief Moves to the homing position.
   * @return True if successful, false otherwise.
   */
  virtual bool goHomingPosition();

  /**
   * @brief Moves to the working position.
   * @return True if successful, false otherwise.
   */
  virtual bool goWorkingPosition();

  /**
   * @brief Gets the ROS loop rate by reference.
   * @return ROS loop rate.
   */
  ros::Rate* getRosLoopRate_();

protected:
  /**
   * @brief Gets the ROS frequency.
   * @return ROS frequency.
   */
  double getRosFrequency_() const;

  /**
   * @brief Gets the ROS node handle.
   * @return ROS node handle.
   */
  ros::NodeHandle getRosNodehandle_() const;

  /**
   * @brief Gets the home joint configuration.
   * @return Home joint configuration.
   */
  std::vector<double> getHomeJoint_() const;

  /**
   * @brief Sets the home joint configuration.
   * @param desiredJoint Desired joint configuration.
   */
  void setHomeJoint_(std::vector<double> desiredJoint);

  /**
   * @brief Pointer to DynamicalSystem instance.
   */
  std::unique_ptr<DynamicalSystem> dynamicalSystem_ = nullptr;

  /**
   * @brief Pointer to IRoboticArmBase instance.
   */
  std::unique_ptr<IRoboticArmBase> roboticArm_ = nullptr;

  /**
   * @brief Pointer to IToolsBase instance.
   */
  std::unique_ptr<IToolsBase> tools_ = nullptr;

  /**
   * @brief Pointer to RosInterfaceNoetic instance.
   */
  std::unique_ptr<RosInterfaceNoetic> rosInterface_ = nullptr;

  /**
   * @brief Moves to a specific point with a linear DS.
   * @param firstQuatPosOffset Vector representing the position offset.
   * @return True if successful, false otherwise.
   */
  bool goToPoint(std::vector<double> firstQuatPosOffset) const;
  /**
   * @brief Take the configuration of the task.
   * @param taskname Name of the task.
   * @return  void
   */
  void takeConfigTask(std::string taskname);

  /**
   * @brief limit cycle speed
   */
  double limitCycleSpeed_;
  /**
   * @brief conergence rate for limit cycle
   */
  double convRate_;
  /**
   * @brief linear speed
   */
  double linearSpeed_;

private:
  /**
   * @brief ROS node handle.
   */
  ros::NodeHandle nh_;

  /**
   * @brief ROS frequency.
   */
  double rosFreq_;

  /**
   * @brief Home joint configuration.
   */
  std::vector<double> homeJoint_;

  /**
   * @brief ROS loop rate.
   */
  ros::Rate loopRate_;
};
