#include "PolygonCoverage.h"

#include <polygon_coverage_msgs/PolygonWithHoles.h>
#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <chrono> // Include this header for std::chrono
#include <string>
#include <thread> // Include this header for std::this_thread

#include "polygon_coverage_msgs/PlannerService.h"
#include "polygon_coverage_msgs/PolygonService.h"

using namespace std;

PolygonCoverage::PolygonCoverage(ros::NodeHandle& n) : nh_(n) {
  // Advertise publishers
  pathPubFlat_ = nh_.advertise<nav_msgs::Path>("/result_path_flat", 10, true);
  pathPubFinal_ = nh_.advertise<nav_msgs::Path>("/result_path_final", 10, true);
  posArraySub_ = nh_.subscribe("/waypoint_list", 10, &PolygonCoverage::poseArrayCallback, this);
}
void PolygonCoverage::setOptimumRad(double rad) { optimumRad = rad; }

void PolygonCoverage::initRosLaunch() {

  // Run roslaunch using the system function in the background
  string launch_command = "roslaunch polygon_coverage_ros coverage_planner.launch";

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

  // // The client should be initialized after the launch has started
  // client.waitForServer(ros::Duration(2.0));
}

void PolygonCoverage::closeRosLaunch() {
  // Use the process ID to terminate the roslaunch process
  string kill_command = "kill " + to_string(launchProcessId);
  int result = system(kill_command.c_str());

  if (result != 0) {
    ROS_ERROR("Failed to terminate roslaunch");
  }
}

void PolygonCoverage::callSetPolygonService(const std::vector<Eigen::Vector3d>& hull_points,
                                            const std::vector<Eigen::Vector3d>& hole_points) {
  // Create a service client
  ros::ServiceClient client = nh_.serviceClient<polygon_coverage_msgs::PolygonService>("/coverage_planner/set_polygon");

  // Create the service request and response objects
  polygon_coverage_msgs::PolygonService srv;

  // Fill in the polygon message
  srv.request.polygon.header.seq = 0;
  srv.request.polygon.header.stamp = ros::Time::now();
  srv.request.polygon.header.frame_id = "base_link";

  // Define the hull points
  for (const auto& point : hull_points) {
    geometry_msgs::Point32 p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    srv.request.polygon.polygon.hull.points.push_back(p);
  }

  // Define the holes points (optional)
  if (!hole_points.empty()) {
    geometry_msgs::Polygon hole_polygon;
    for (const auto& point : hole_points) {
      geometry_msgs::Point32 p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      hole_polygon.points.push_back(p);
    }
    srv.request.polygon.polygon.holes.push_back(hole_polygon);
  }

  // Call the service
  if (client.call(srv)) {
    ROS_INFO("Service call successful");
  } else {
    ROS_ERROR("Failed to call service /coverage_planner/set_polygon");
  }
}
void PolygonCoverage::callStartService(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
  // Create a service client
  ros::ServiceClient client = nh_.serviceClient<polygon_coverage_msgs::PlannerService>("/coverage_planner/plan_path");

  // Create the service request and response objects
  polygon_coverage_msgs::PlannerService srv;

  // Fill in the start_pose
  srv.request.start_pose.header.frame_id = "base_link";
  srv.request.start_pose.pose.position.x = start.x();
  srv.request.start_pose.pose.position.y = start.y();
  srv.request.start_pose.pose.position.z = start.z();
  srv.request.start_pose.pose.orientation.x = 0.0;
  srv.request.start_pose.pose.orientation.y = 0.0;
  srv.request.start_pose.pose.orientation.z = 0.0;
  srv.request.start_pose.pose.orientation.w = 1.0;

  // Fill in the goal_pose
  srv.request.goal_pose.header.frame_id = "base_link";
  srv.request.goal_pose.pose.position.x = goal.x();
  srv.request.goal_pose.pose.position.y = goal.y();
  srv.request.goal_pose.pose.position.z = goal.z();
  srv.request.goal_pose.pose.orientation.x = 0.0;
  srv.request.goal_pose.pose.orientation.y = 0.0;
  srv.request.goal_pose.pose.orientation.z = 0.0;
  srv.request.goal_pose.pose.orientation.w = 1.0;

  // Call the service
  if (client.call(srv)) {
    ROS_INFO("Service call successful");
    // Process the response if needed
  } else {
    ROS_ERROR("Failed to call service /coverage_planner/plan_path");
  }
}

nav_msgs::Path PolygonCoverage::convertPoseArrayToPath(const geometry_msgs::PoseArray& pose_array) {
  path_.header = pose_array.header;

  for (const auto& pose : pose_array.poses) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = pose_array.header;
    pose_stamped.pose = pose;
    path_.poses.push_back(pose_stamped);
  }
  pathPubFlat_.publish(path_);
  return path_;
}

void PolygonCoverage::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
  convertPoseArrayToPath(*msg);
  ROS_INFO("Received path with %lu poses", msg->poses.size());
  // Publish the path or do further processing with it
}

void PolygonCoverage::publishNavmsg(nav_msgs::Path path) {
  ros::spinOnce();
  pathPubFinal_.publish(path);
}
