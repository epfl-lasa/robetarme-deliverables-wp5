// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include <ros/ros.h>

#include <tuple>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "TaskSurfaceFinishing.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  double rosFreq = 300;

  // Init ros
  ros::init(argc, argv, "task_surface");
  ros::NodeHandle nh;

  ros::Rate loop_rate(300);

  // Init class for Tasks --------------------------------
  unique_ptr<Tasks> tasks = nullptr;
  tasks = make_unique<Tasks>(nh, rosFreq);

  // comput path -----------------------------------------
  tasks->computePathShotcrete();

  // Init surface finishing
  bool valid = tasks->initTask("surface_finishing");
  // tasks->goHome();

  if (valid) {
    cout << "Iniitalization surface_finishing  ok" << endl;
  } else {
    cout << "Iniitalization surface_finishing  failed" << endl;
    return 0;
  }

  tasks->TestSF();

  return 0;
}
