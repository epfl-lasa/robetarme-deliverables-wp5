// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include "IRoboticArmBase.h"
#include <ros/ros.h>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "PathPlanner.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"

class Tasks {
public:
  Tasks(ros::NodeHandle& n, double freq);
  bool computePathShotcrete();
  bool goFirstPosition();
  bool DoShotcrete();
  bool initTask(std::string taskName);
  bool goHome();
  void setHome(std::vector<double> desiredJoint);
  bool TestSF();


  bool checkInit = false;
  bool checkFirstPosition = false;
  bool checkFinish = false;
  bool checkPath = false;
  bool checkGoHome = false;

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
