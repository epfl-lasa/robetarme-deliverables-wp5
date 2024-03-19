#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <geometry_msgs/Point.h>        //<----------to remove
#include <geometry_msgs/PointStamped.h> //<----------to remove
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RoboticArmIiwa7.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "visualization_msgs/Marker.h"

enum TaskType : int8_t {
  TASK_UNDEFINED = -1,
  SHOTCRETE,
  SURFACE_FINISHING,
  SAND_BLASTING,
  MAM_REBARS,
  NB_TASKS // Keep at the end of enum => number of types
};

class ITaskBase {
public:
  inline static const std::map<std::string, TaskType> taskTypes{{"shotcrete", SHOTCRETE},
                                                                {"surface_finishing", SURFACE_FINISHING},
                                                                {"sand_blasting", SAND_BLASTING},
                                                                {"mam_rebars", MAM_REBARS}};

  bool checkInitialization = false;
  bool checkFinish = false;
  bool checkPath = false;

  bool checkHomingPosition = false;
  bool checkWorkingPosition = false;

  ITaskBase(ros::NodeHandle& nh, double freq);

  virtual bool initialize(std::string robotName) = 0;
  virtual bool computePath() = 0;
  virtual bool execute() = 0;

  virtual bool goHomingPosition() = 0;
  virtual bool goWorkingPosition() = 0;

  virtual void setHomingPosition(std::vector<double> desiredJoint) = 0;

protected:
  ros::NodeHandle nh_;
  ros::Rate loopRate_;

  double rosFreq_;
  std::vector<double> homeJoint_;

  // Create an unique pointer for the instance of IRoboticArmBase
  std::unique_ptr<IRoboticArmBase> roboticArm_ = nullptr;

  // Create an unique pointer for the instance of RosInterfaceNoetic
  std::unique_ptr<RosInterfaceNoetic> rosInterface_ = nullptr;
};
