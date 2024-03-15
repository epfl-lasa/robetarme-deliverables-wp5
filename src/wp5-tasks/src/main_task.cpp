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
#include "TaskShotcrete.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  bool valid = false;
  double deltaTime = 0.001;
  double rosFreq = 1 / deltaTime;

  // Init ros
  ros::init(argc, argv, "main_tasks");
  ros::NodeHandle nh;
  ros::Rate loopRate(1 / deltaTime);

  // Get task configuration
  string yamlPath = string(WP5_TASKS_DIR) + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);
  string robotName = config["shotcrete"]["robot_name"].as<string>();

  // Create an unique pointer for the instance of RosInterfaceNoetic
  // rosInterface = make_unique<RosInterfaceNoetic>(nh, robotName);

  // Init class for Tasks
  unique_ptr<TaskShotcrete> task = nullptr;
  task = make_unique<TaskShotcrete>(nh, rosFreq);

  // Compute path
  task->computePath();

  // Init task
  valid = task->initialize(robotName);
  if (valid) {
    cout << "Iniitalization shotcrete ok" << endl;
  } else {
    cout << "Iniitalization shotcrete failed" << endl;
    return 0;
  }

  // Go working position
  valid = task->goWorkingPosition();
  if (valid) {
    cout << "Robot in working position" << endl;
  } else {
    cout << "Failed to go into working position" << endl;
    return 0;
  }

  // Do task
  valid = task->execute();
  if (valid) {
    cout << "Task Done" << endl;
  } else {
    cout << "Task Failed : go to homing position" << endl;
    valid = task->goHomingPosition();

    return 0;
  }

  return 0;
}
