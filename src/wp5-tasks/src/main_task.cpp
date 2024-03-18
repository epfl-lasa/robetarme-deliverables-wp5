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
#include "TaskSafeFSM.h"
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
  shared_ptr<TaskShotcrete> task = make_shared<TaskShotcrete>(nh, rosFreq);

  // Compute path
  // task->computePath();

  std::unique_ptr<msm::back::state_machine<TaskSafeFSM>> internalFSM_ =
      make_unique<msm::back::state_machine<TaskSafeFSM>>(task);

  // Initialize and test the FSM
  internalFSM_->start();
  internalFSM_->process_event(Initialized());
  internalFSM_->process_event(PathComputed());
  internalFSM_->process_event(SafetyTrigger());
  internalFSM_->process_event(Recover());
  internalFSM_->process_event(Start());
  internalFSM_->process_event(Finished());
  internalFSM_->process_event(SafetyTrigger());
  internalFSM_->process_event(Recover());

  // // Init task
  // valid = task->initialize(robotName);
  // if (valid) {
  //   cout << "Iniitalization shotcrete ok" << endl;
  // } else {
  //   cout << "Iniitalization shotcrete failed" << endl;
  //   return 0;
  // }

  // // Go working position
  // valid = task->goWorkingPosition();
  // if (valid) {
  //   cout << "Robot in working position" << endl;
  // } else {
  //   cout << "Failed to go into working position" << endl;
  //   return 0;
  // }

  // // Do task
  // valid = task->execute();
  // if (valid) {
  //   cout << "Task Done" << endl;
  // } else {
  //   cout << "Task Failed : go to homing position" << endl;
  //   valid = task->goHomingPosition();

  //   return 0;
  // }

  return 0;
}
