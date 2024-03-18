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

int main(int argc, char** argv) {

  double deltaTime = 0.001;
  double rosFreq = 1 / deltaTime;
  // init ros
  ros::init(argc, argv, "task_surface");
  ros::NodeHandle nh;

  ros::Rate loop_rate(1 / deltaTime);

  //init class for Tasks -----------------------------------------
  unique_ptr<Tasks> tasks = nullptr;
  tasks = make_unique<Tasks>(nh, rosFreq);

  // comput path -----------------------------------------
  tasks->computePathShotcrete();

  //init shotcrete
  bool valid = tasks->initTask("surface_finishing");
  // tasks->goHome();

  if (valid) {
    cout << "Iniitalization surface_finishing  ok" << endl;
  } else {
    cout << "Iniitalization surface_finishing  failed" << endl;
    return 0;
  }

  tasks->set_bias();
  

  tasks->TestSF();

  return 0;
}
