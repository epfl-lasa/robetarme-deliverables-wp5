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

  //TODO(Tristan): delete rviz dependency
  ros::Publisher point_pub;
  ros::Publisher pub_desired_vel_filtered;

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
