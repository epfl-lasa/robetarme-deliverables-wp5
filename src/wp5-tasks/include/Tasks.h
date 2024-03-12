// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include <ros/ros.h>
#include "IRoboticArmBase.h"

#include "DynamicalSystem.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "PathPlanner.h"
#include "BoustrophedonServer.h"

class Tasks {
public:
  Tasks(ros::NodeHandle& n, double freq);
  bool computePathShotcrete();
  bool goFirstPosition();
  bool DoShotcrete();
  bool initShotcrete();
  bool goHome();
  void setHome(std::vector<double> desiredJoint);

  bool checkInit = false;
  bool checkFirstPosition = false;
  bool checkFinish = false;
  bool checkPath = false;
  bool checkGoHome = false;

private:
  ros::NodeHandle nh;
  ros::Rate loop_rate;
  std::vector<double> homeJoint;


  double rosFreq;
  // Create an unique pointer for the instance of IRoboticArmBase
  std::unique_ptr<IRoboticArmBase> roboticArm = nullptr;
  // Create an unique pointer for the instance of RosInterfaceNoetic
  std::unique_ptr<RosInterfaceNoetic> rosInterface = nullptr;
  // Create an unique pointer for the instance of DynamicalSystem
  std::unique_ptr<DynamicalSystem> dynamicalSystem = nullptr;
  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetextraction = nullptr;
  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathplanner = nullptr;
  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<BoustrophedonServer> boustrophedonserver = nullptr;
  

  };
