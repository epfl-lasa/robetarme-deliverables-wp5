#pragma once

#include <actionlib/client/simple_action_client.h>
#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <ros/ros.h>
#include <sys/types.h>

class BoustrophedonServer {
public:
  BoustrophedonServer(ros::NodeHandle& n);
  void initRosLaunch();
  void closeRosLaunch();
  void setOptimumRad(double rad);
  ros::Publisher polygon_pub;
  ros::Publisher path_pub;
  ros::Publisher start_pub;
  actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client;

private:
  ros::NodeHandle nh;
  double optimumRad;
  pid_t launchProcessId; // To store the PID of the roslaunch process
};
