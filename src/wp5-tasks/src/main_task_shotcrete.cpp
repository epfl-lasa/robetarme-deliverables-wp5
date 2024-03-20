// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <tuple>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "TaskFSM.h"
#include "TaskShotcrete.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  bool valid = false;
  double deltaTime = 0.001;
  double rosFreq = 1 / deltaTime;
  // init ros
  ros::init(argc, argv, "task_shotcrete");
  ros::NodeHandle nh;
  ros::Rate loopRate(1 / deltaTime);

  // Get task configuration
  string yamlPath = string(WP5_TASKS_DIR) + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);
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

  string robotName = config["shotcrete"]["robot_name"].as<string>();

  // Create an unique pointer for the instance of TaskFSM
  std::shared_ptr<TaskShotcrete> taskShotcrete = std::make_shared<TaskShotcrete>(nh, rosFreq, robotName);

  taskFsm_ internalFSM_(taskShotcrete);

  // Initialize and test the FSM
  internalFSM_.start();
  internalFSM_.process_event(Start());

  return 0;
}
