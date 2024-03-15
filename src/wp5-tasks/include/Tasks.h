#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include <ros/ros.h>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"

class Tasks {
public:
  enum Type { UNDEFINED, SHOTCRETE, SURFACE_FINISHING, SAND_BLASTING, MAM_REBARS };

  bool checkInit = false;
  bool checkWorkingPosition = false;
  bool checkFinish = false;
  bool checkPath = false;
  bool checkGoHome = false;

  Tasks(ros::NodeHandle& n, double freq);

  bool initialize() = 0;
  bool execute() = 0;

  bool computePath() = 0;
  bool goHomingPosition() = 0;
  bool goWorkingPosition() = 0;

  void setHomingPosition(std::vector<double> desiredJoint);

private:
  ros::NodeHandle nh_;
  ros::Rate loopRate_;
  std::vector<double> homeJoint_;

  //TODO: delet rviz dependency
  ros::Publisher pointPub_;
  ros::Publisher pubDesiredVelFiltered_;

  double rosFreq_;
  // Create an unique pointer for the instance of IRoboticArmBase
  std::unique_ptr<IRoboticArmBase> roboticArm_ = nullptr;
  // Create an unique pointer for the instance of RosInterfaceNoetic
  std::unique_ptr<RosInterfaceNoetic> rosInterface_ = nullptr;
  // Create an unique pointer for the instance of DynamicalSystem
  std::unique_ptr<DynamicalSystem> dynamicalSystem_ = nullptr;
  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetextraction_ = nullptr;
  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathplanner_ = nullptr;
  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<BoustrophedonServer> boustrophedonserver_ = nullptr;
};
