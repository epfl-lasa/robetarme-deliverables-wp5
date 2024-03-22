#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ros/ros.h>

#include <memory>
#include <string>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "ITaskBase.h"
#include "PathPlanner.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"

class TaskSurfaceFinishing : public ITaskBase {
public:
  TaskSurfaceFinishing(ros::NodeHandle& n, double freq, std::string robotName);

  bool computePath();
  bool execute();

  bool TestSF();
  void set_bias();

  std::vector<double> homeJoint_;
  std::vector<double> biasWrench_;
  Eigen::VectorXd outputTwist_;

private:
  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetExtraction_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathPlanner_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<BoustrophedonServer> boustrophedonServer_ = nullptr;

  Eigen::VectorXd decoderWrench();
};
