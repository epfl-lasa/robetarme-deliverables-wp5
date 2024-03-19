#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ros/ros.h>

#include <memory>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "ITaskBase.h"
#include "PathPlanner.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"

class TaskShotcrete : public ITaskBase {
public:
  TaskShotcrete(ros::NodeHandle& n, double freq);

  bool initialize(std::string robotName);
  bool execute();
  bool computePath();

  bool goHomingPosition();
  bool goWorkingPosition();

  void setHomingPosition(std::vector<double> desiredJoint);

private:
  // Create an unique pointer for the instance of DynamicalSystem
  std::unique_ptr<DynamicalSystem> dynamicalSystem_ = nullptr;

  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetExtraction_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathPlanner_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<BoustrophedonServer> boustrophedonServer_ = nullptr;
};
