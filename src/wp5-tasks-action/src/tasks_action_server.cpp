#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <string>
#include "Tasks.h"
#include <wp5_tasks_action/ExecuteTaskAction.h>

using namespace std;

class ExecuteTaskAction {
public:
  explicit ExecuteTaskAction() : as_(nh_, name, boost::bind(&ExecuteTaskAction::executeCB, this, _1), false) {
    as_.start();
  }

  void executeCB(const wp5_tasks_action::ExecuteTaskGoalConstPtr& goal) {
    current_task_type_ = static_cast<Tasks::Type>(goal->task_type);
  }

private:
  ros::NodeHandle nh_;
  Tasks::Type current_task_type_ = Tasks::Type::UNDEFINED;

  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<wp5_tasks_action::ExecuteTaskAction> as_;

  // create messages that are used to published feedback/result
  wp5_tasks_action::ExecuteTaskFeedback feedback_;
  wp5_tasks_action::ExecuteTaskResult result_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ExecuteTask");

  ExecuteTaskAction executeTask("ExecuteTask");
  ros::spin();

  return 0;
}
