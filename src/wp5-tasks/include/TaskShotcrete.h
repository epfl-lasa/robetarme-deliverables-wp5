#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ros/ros.h>

#include <memory>
#include <string>

#include "PolygonCoverage.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "ITaskBase.h"
#include "IToolsBase.h"
#include "PathPlanner.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"

class TaskShotcrete : public ITaskBase {
public:
  TaskShotcrete(ros::NodeHandle& n, double freq, std::string robotName);

  bool computePath();
  bool execute();

private:
  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetExtraction_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathPlanner_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PolygonCoverage> polygonCoverage_ = nullptr;
};
