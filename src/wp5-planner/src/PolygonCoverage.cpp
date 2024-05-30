#include "PolygonCoverage.h"

#include <polygon_coverage_msgs/PolygonWithHoles.h>
#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>
#include <pybind11/embed.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <chrono> // Include this header for std::chrono
#include <fstream>
#include <iostream>
#include <string>
#include <thread> // Include this header for std::this_thread

#include "polygon_coverage_msgs/PlannerService.h"
#include "polygon_coverage_msgs/PolygonService.h"

using namespace std;
namespace py = pybind11;

PolygonCoverage::PolygonCoverage(ros::NodeHandle& n) : nh_(n) {
  // Advertise publishers
  PolygonFlatPub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/original_polygon", 1, true);
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

void PolygonCoverage::convertPoseArrayToPath(const geometry_msgs::PoseArray& pose_array) {
  path_.header = pose_array.header;

  for (const auto& pose : pose_array.poses) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = pose_array.header;
    pose_stamped.pose = pose;
    path_.poses.push_back(pose_stamped);
  }
  pathPubFlat_.publish(path_);
}

nav_msgs::Path PolygonCoverage::getPathFromPolygonFlat() { return path_; }

void PolygonCoverage::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
  convertPoseArrayToPath(*msg);
  ROS_INFO("Received path with %lu poses", msg->poses.size());
  // Publish the path or do further processing with it
}

void PolygonCoverage::publishNavmsg(nav_msgs::Path path) {
  ros::spinOnce();
  pathPubFinal_.publish(path);
}

// Function to write the positions from a nav_msgs::Path to a file
void PolygonCoverage::writePathToFile(const nav_msgs::Path& path, const std::string& file_path) {
  std::ofstream outputFile(file_path);

  if (!outputFile.is_open()) {
    ROS_ERROR("Unable to open file for writing.");
    return;
  }

  for (const auto& pose : path.poses) {
    const geometry_msgs::Point& position = pose.pose.position;
    outputFile << position.x << " " << position.y << " " << position.z << "\n";
  }

  outputFile.close();
}

void PolygonCoverage::seePolygonFlat(std::vector<Eigen::Vector3d> polygonsPositions) {

  geometry_msgs::PolygonStamped visualpolygonTarget;
  visualpolygonTarget.header.frame_id = "base_link";
  visualpolygonTarget.header.stamp = ros::Time::now();

  for (const auto& point : polygonsPositions) {
    geometry_msgs::Point32 msg_point;
    msg_point.x = point.x();
    msg_point.y = point.y();
    msg_point.z = point.z();
    visualpolygonTarget.polygon.points.push_back(msg_point);
  }
  PolygonFlatPub_.publish(visualpolygonTarget);
}

//TODO : add quaternion to the navmsgs::Path
nav_msgs::Path PolygonCoverage::convertFileToNavMsgsPath() {
  string file_path = string(WP5_PLANNER_DIR) + "/data/paths/waypointInOriSpace.txt";
  string frame_id = "base_link";
  nav_msgs::Path path;
  path.header.frame_id = frame_id;

  std::ifstream infile(file_path);
  if (!infile.is_open()) {
    ROS_ERROR("Unable to open file: %s", file_path.c_str());
    return path;
  }

  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    double x, y, z;
    if (!(iss >> x >> y >> z)) {
      ROS_WARN("Invalid line format: %s", line.c_str());
      continue; // Skip invalid lines
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = z;
    pose_stamped.pose.orientation.w = 1.0; // Default orientation

    path.poses.push_back(pose_stamped);
  }

  infile.close();
  return path;
}
bool PolygonCoverage::pointCloudTransformer() {
  bool success = false; // Default value indicating failure

  py::scoped_interpreter guard{}; // Start the interpreter and keep it alive

  try {
    // Run the Python script
    py::object result = py::eval<py::eval_statements>(R"(
            import sys
            import rospkg

            # Initialize the ROS package manager
            rospack = rospkg.RosPack()

            # Get the path of a specific ROS package
            package_path = rospack.get_path('wp5_planner')

            sys.path.insert(0, package_path + "/scripts")  # Adjust the path to your script location if necessary
            import point_cloud_transformer
            result = point_cloud_transformer.main()
        )");

    // Convert the Python result to a C++ boolean directly
    success = py::bool_(result);
  } catch (const py::error_already_set& e) {
    std::cerr << "Python error: " << e.what() << std::endl;
  }

  return success; // Return the result
}

// Calculate the perpendicular distance from a point to a line
double PolygonCoverage::perpendicularDistance(const Eigen::Vector3d& point,
                                              const Eigen::Vector3d& lineStart,
                                              const Eigen::Vector3d& lineEnd) {
  Eigen::Vector3d line = lineEnd - lineStart;
  Eigen::Vector3d pointLineStart = point - lineStart;
  Eigen::Vector3d crossProduct = line.cross(pointLineStart);
  return crossProduct.norm() / line.norm();
}

// Ramer-Douglas-Peucker algorithm
std::vector<Eigen::Vector3d> PolygonCoverage::rdp(const std::vector<Eigen::Vector3d>& points, double epsilon) {
  std::vector<Eigen::Vector3d> result;
  if (points.size() < 2) {
    throw std::invalid_argument("Not enough points to simplify");
  }

  double dmax = 0.0;
  size_t index = 0;
  size_t end = points.size() - 1;

  for (size_t i = 1; i < end; ++i) {
    double d = perpendicularDistance(points[i], points[0], points[end]);
    if (d > dmax) {
      index = i;
      dmax = d;
    }
  }

  if (dmax > epsilon) {
    std::vector<Eigen::Vector3d> recResults1;
    std::vector<Eigen::Vector3d> recResults2;
    std::vector<Eigen::Vector3d> firstLine(points.begin(), points.begin() + index + 1);
    std::vector<Eigen::Vector3d> lastLine(points.begin() + index, points.end());

    recResults1 = rdp(firstLine, epsilon);
    recResults2 = rdp(lastLine, epsilon);

    result.assign(recResults1.begin(), recResults1.end() - 1);
    result.insert(result.end(), recResults2.begin(), recResults2.end());
  } else {
    result.clear();
    result.push_back(points[0]);
    result.push_back(points[end]);
  }

  return result;
}

vector<vector<double>> PolygonCoverage::convertNavPathToVectorVector(const nav_msgs::Path& inputPath) {

  size_t size = inputPath.poses.size();
  vector<vector<double>> path;

  for (size_t i = 0; i < size; i++) {
    geometry_msgs::PoseStamped pose = inputPath.poses[i];
    vector<double> quatPos;

    quatPos.push_back(pose.pose.orientation.x);
    quatPos.push_back(pose.pose.orientation.y);
    quatPos.push_back(pose.pose.orientation.z);
    quatPos.push_back(pose.pose.orientation.w);

    quatPos.push_back(pose.pose.position.x);
    quatPos.push_back(pose.pose.position.y);
    quatPos.push_back(pose.pose.position.z);

    path.push_back(quatPos);
  }

  return path;
}
vector<Eigen::Vector3d> PolygonCoverage::getFlatPolygonFromTxt() {

  // Extract polygons for boustrophedon
  ifstream inputFile(string(WP5_PLANNER_DIR) + "/data/polygons/boundary_planeData_curved.txt");
  vector<Eigen::Vector3d> polygonsPositions;

  if (!inputFile.is_open()) {
    cerr << "Unable to open file" << endl;
    return polygonsPositions;
  }

  int i_decrease = 0;
  double x, y, z;
  while (inputFile >> x >> y >> z) {
    if ((i_decrease % 2) == 0) {
      Eigen::Vector3d point(x, y, z);
      polygonsPositions.push_back(point);
    }
    i_decrease++;
  }
  inputFile.close();
  return polygonsPositions;
}

//TODO: fill the function with RUIs code
void PolygonCoverage::getPathFromFeatureSpaceToRealSpace() {}
//TODO: fill the function with RUIs code
void PolygonCoverage::convertPclToPolygon() {}
//TODO: fill the function with RUIs code
void PolygonCoverage::featureSpaceAlgorithm() {}
