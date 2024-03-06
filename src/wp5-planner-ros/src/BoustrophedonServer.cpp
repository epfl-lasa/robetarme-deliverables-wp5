#include "BoustrophedonServer.h"


BoustrophedonServer::BoustrophedonServer(ros::NodeHandle& n, double rad) : nh(n), client("plan_path", true), optimumRad(rad) {
  // Advertise publishers
  polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("/input_polygon", 1, true);
  path_pub = nh.advertise<nav_msgs::Path>("/result_path", 1, true);
  start_pub = nh.advertise<geometry_msgs::PoseStamped>("/start_pose", 1, true);

  // Set parameter
  nh.setParam("/boustrophedon_server/stripe_separation",  optimumRad);

  // Run roslaunch using the system function in the background
  std::string launch_command = "roslaunch boustrophedon_server boustrophedon_server.launch";
  launchProcessId = fork(); // Fork a new process for roslaunch

  if (launchProcessId == 0) {
      // This is the child process (roslaunch)
      system(launch_command.c_str());
      exit(0); // Terminate the child process after roslaunch is done
  }

  // Sleep for a short duration to allow the launch to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // The client should be initialized after the launch has started
  client.waitForServer(ros::Duration(5.0));
}


void BoustrophedonServer::closeRosLaunch() {
    // Use the process ID to terminate the roslaunch process
    std::string kill_command = "kill " + std::to_string(launchProcessId);
    system(kill_command.c_str());
}
