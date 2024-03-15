#include "TargetExtraction.h"

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

//TargetExrtaction functon
TargetExtraction::TargetExtraction(ros::NodeHandle& nh) :
    poseTargetSub_(
        nh.subscribe("/vrpn_client_node/TargetRobetarme/pose_transform", 10, &TargetExtraction::CCVrpnTarget, this)) {

  originalPolygonPub_ = nh.advertise<geometry_msgs::PolygonStamped>("/original_polygon", 1, true);
  // Get the path to the package
  string package_path =
      ros::package::getPath("wp5-planner-ros"); // Replace "your_package" with your actual package name

  // Load parameters from YAML file
  string yaml_path = package_path + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);

  // Access parameters from the YAML file
  heightTarget_ = config["heightTarget"].as<double>();
  widthTarget_ = config["widthTarget"].as<double>();

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
  visualpolygonTarget.header.frame_id = "base";
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
