#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <string>
#include <wp5_tasks_action/ExecuteTaskAction.h>

using namespace std;

class ExecuteTaskAction {
protected:
  ros::NodeHandle nh_;
  std::string action_name_;

  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<wp5_tasks_action::ExecuteTaskAction> as_;

  // create messages that are used to published feedback/result
  wp5_tasks_action::ExecuteTaskFeedback feedback_;
  wp5_tasks_action::ExecuteTaskResult result_;

public:
  explicit ExecuteTaskAction(std::string name) :
      as_(nh_, name, boost::bind(&ExecuteTaskAction::executeCB, this, _1), false), action_name_(name) {
    as_.start();
  }

  void executeCB(const wp5_tasks_action::ExecuteTaskGoalConstPtr& goal) {
    bool success = true;

    cout << "Task ID: " << goal->task_id << endl;

    // start executing the action
    ROS_INFO("Executing action-task %s", to_string(goal->task_id).c_str());

    feedback_.percent_complete = 0;
    as_.publishFeedback(feedback_); // publish the feedback

    if (success) {
      result_.success = success;
      ROS_INFO("%s: Succeeded", to_string(goal->task_id).c_str());

      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ExecuteTask");

  ExecuteTaskAction executeTask("ExecuteTask");
  ros::spin();

  return 0;
}
