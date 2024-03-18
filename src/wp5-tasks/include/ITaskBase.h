#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ros/ros.h>

#include <map>
#include <memory>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"

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

  ITaskBase(ros::NodeHandle& n, double freq);
  bool initialize(std::string robotName);

protected:
  bool execute();
  bool computePath();

  bool goHomingPosition();
  bool goWorkingPosition();

  void setHomingPosition(std::vector<double> desiredJoint);

  // Create an unique pointer for the instance of RosInterfaceNoetic
  std::unique_ptr<RosInterfaceNoetic> rosInterface = nullptr;

private:
  ros::NodeHandle nh_;
  ros::Rate loopRate_;
  std::vector<double> homeJoint_;

  //TODO(Tristan): delete rviz dependency
  ros::Publisher pointPub_;
  ros::Publisher desiredVelFilteredPub_;

  double rosFreq_;

  // Create an unique pointer for the instance of IRoboticArmBase
  std::unique_ptr<IRoboticArmBase> roboticArm_ = nullptr;

  // Create an unique pointer for the instance of RosInterfaceNoetic
  std::unique_ptr<RosInterfaceNoetic> rosInterface_ = nullptr;

  // Create an unique pointer for the instance of DynamicalSystem
  std::unique_ptr<DynamicalSystem> dynamicalSystem_ = nullptr;

  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetExtraction_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathPlanner_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<BoustrophedonServer> boustrophedonServer_ = nullptr;
};
