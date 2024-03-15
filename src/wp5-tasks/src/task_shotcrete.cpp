// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "Tasks.h"
#include <ros/ros.h>
#include <tuple>

using namespace std;
using namespace Eigen;

int shotcrete(ros::NodeHandle nh, double rosFreq = 300) {

  ros::Rate loop_rate(1 / deltaTime);

  //init class for Tasks -----------------------------------------
  unique_ptr<Tasks> tasks = nullptr;
  tasks = make_unique<Tasks>(nh, rosFreq);

  // comput path -----------------------------------------
  tasks->computePathShotcrete();

  //init shotcrete
  bool valid = tasks->initTask("shotcrete");
  // tasks->goHome();

  if (valid) {
    cout << "Iniitalization shotcrete  ok" << endl;
  } else {
    cout << "Iniitalization shotcrete  failed" << endl;
    return 0;
  }

  // go first position
  valid = tasks->goFirstPosition();
  if (valid) {
    cout << "We are in the first position" << endl;
  } else {
    cout << "failed to go into first position" << endl;
    return 0;
  }

  // tasks->goHome();
  // tasks->goFirstPosition();

  // Do shotcrete
  valid = tasks->DoShotcrete();
  if (valid) {
    cout << "shotcrete Done" << endl;
  } else {
    cout << "failed to perform shotcrete, go to home" << endl;
    valid = tasks->goHome();

    return 0;
  }

  return 0;
}
