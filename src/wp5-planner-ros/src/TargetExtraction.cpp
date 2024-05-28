#include "TargetExtraction.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

//TargetExrtaction functon
TargetExtraction::TargetExtraction(ros::NodeHandle& nh) {

  originalPolygonPub_ = nh.advertise<geometry_msgs::PolygonStamped>("/original_polygon", 1, true);

  // Load parameters from YAML file
  string alternativeYamlPath = string(WP5_PLANNER_ROS_DIR) + "/config/target_config.yaml";
  string yamlPath = string(WP5_PLANNER_ROS_DIR) + "/../../config/target_config.yaml";

  // Check if the alternative YAML file exists
  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << yamlPath << endl;
  } else {
    yamlPath = alternativeYamlPath;
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);

  // Access parameters from the YAML file
  heightTarget_ = config["heightTarget"].as<double>();
  widthTarget_ = config["widthTarget"].as<double>();
  string nameTarget = config["nameTarget"].as<string>();

  poseTargetSub_ = nh.subscribe(nameTarget, 10, &TargetExtraction::CCVrpnTarget, this);

  cout << "Waiting for target Pose" << endl;

  while (!targetReceived_ && ros::ok()) {
    ros::spinOnce();
  }
  cout << "rostopic for the target received" << endl;
}

vector<Vector3d> TargetExtraction::getPolygons() {
  // Desired displacements
  vector<Vector3d> displacements{Vector3d(widthTarget_ / 2.0, heightTarget_ / 2.0, 0),
                                 Vector3d(-widthTarget_ / 2.0, heightTarget_ / 2.0, 0),
                                 Vector3d(-widthTarget_ / 2.0, -heightTarget_ / 2.0, 0),
                                 Vector3d(widthTarget_ / 2.0, -heightTarget_ / 2.0, 0)};

  // Extract position and ensure quaternion is normalized
  Vector3d position = targetPos_;
  Quaterniond normalizedQuat = targetQuat_.normalized();
  Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

  // Calculate new positions
  polygonsPositions_.clear(); // Clear existing positions
  for (const auto& displacement : displacements) {
    polygonsPositions_.push_back(position + rotation_matrix * displacement);
  }
  cout << "Polygons well computed" << endl;
  seeTarget();
  return polygonsPositions_;
}

void TargetExtraction::setPolygons(std::vector<Eigen::Vector3d> positions) {
  polygonsPositions_ = positions;

  //compute center
  Eigen::Vector3d centroid(0.0, 0.0, 0.0);
  for (const auto& pos : positions) {
    centroid += pos;
  }
  centroid /= positions.size();
  targetPos_ = centroid;

  // compute orientatino

  Eigen::MatrixXd centered(positions.size(), 3);
  for (size_t i = 0; i < positions.size(); ++i) {
    centered.row(i) = positions[i] - centroid;
  }

  // Compute the covariance matrix
  Eigen::Matrix3d covariance = centered.transpose() * centered;

  // Perform Singular Value Decomposition (SVD)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d orientation = svd.matrixU(); // The matrix of the left singular vectors

  // Convert the orientation matrix to a quaternion
  Eigen::Quaterniond quaternion(orientation);

  targetQuat_ = quaternion;
}

Quaterniond TargetExtraction::getQuatTarget() { return targetQuat_; }

Vector3d TargetExtraction::getPosTarget() { return targetPos_; }

void TargetExtraction::CCVrpnTarget(const geometry_msgs::PoseStamped::ConstPtr msg) {
  targetPos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  targetQuat_ =
      Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  targetReceived_ = true;
}

void TargetExtraction::seeTarget() {

  geometry_msgs::PolygonStamped visualpolygonTarget;
  visualpolygonTarget.header.frame_id = "base_link";
  visualpolygonTarget.header.stamp = ros::Time::now();

  for (const auto& point : polygonsPositions_) {
    geometry_msgs::Point32 msg_point;
    msg_point.x = point.x();
    msg_point.y = point.y();
    msg_point.z = point.z();
    visualpolygonTarget.polygon.points.push_back(msg_point);
  }
  originalPolygonPub_.publish(visualpolygonTarget);
}

nav_msgs::Path TargetExtraction::convertFileToPath() {
  string file_path = string(WP5_PLANNER_ROS_DIR) + "/data/txts/waypointInOriSpace.txt";
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