#include <actionlib/client/simple_action_client.h>
#include <wp5_tasks_action/ExecuteTaskAction.h>

typedef actionlib::SimpleActionClient<wp5_tasks_action::ExecuteTaskAction> Client;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ExecuteTask");
  Client client("ExecuteTask", true); // true -> don't need ros::spin()
  client.waitForServer();

  wp5_tasks_action::ExecuteTaskGoal goal;
  goal.task_id = 1;

  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    printf("Yay! The task is performed well");
  }

  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}