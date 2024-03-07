#include "BoustrophedonServer.h"

#include <chrono> // Include this header for std::chrono
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread> // Include this header for std::this_thread

using namespace std;

BoustrophedonServer::BoustrophedonServer(ros::NodeHandle& n) : nh(n), client("plan_path", true) {
  // Advertise publishers
  polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("/input_polygon", 1, true);
  path_pub = nh.advertise<nav_msgs::Path>("/result_path", 1, true);
  start_pub = nh.advertise<geometry_msgs::PoseStamped>("/start_pose", 1, true);
}
void BoustrophedonServer::setOptimumRad(double rad) { optimumRad = rad; }

void BoustrophedonServer::initRosLaunch() {

  // Set parameter
  nh.setParam("/boustrophedon_server/stripe_separation", optimumRad);

  // Run roslaunch using the system function in the background
  string launch_command = "roslaunch boustrophedon_server boustrophedon_server.launch";

  // Fork a new process for roslaunch
  launchProcessId = fork();

  if (launchProcessId == 0) {
    // This is the child process (roslaunch)
    int result = system(launch_command.c_str());
    if (result != 0) {
      ROS_ERROR("Failed to start roslaunch");
      exit(1);
    }

    // Terminate the child process after roslaunch is done
    exit(0);
  }

  // Sleep for a short duration to allow the launch to initialize
  this_thread::sleep_for(chrono::seconds(2));

  // The client should be initialized after the launch has started
  client.waitForServer(ros::Duration(2.0));
}

void BoustrophedonServer::closeRosLaunch() {
  // Use the process ID to terminate the roslaunch process
  string kill_command = "kill " + to_string(launchProcessId);
  int result = system(kill_command.c_str());

  if (result != 0) {
    ROS_ERROR("Failed to terminate roslaunch");
  }
}
