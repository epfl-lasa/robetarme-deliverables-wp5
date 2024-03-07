#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include <vector>

// this class extrac the polygon and the Pose center of the Target
// it should be replace by the same extraction but from a CAD
class TargetExtraction {
public:
  TargetExtraction(ros::NodeHandle& nh);
  std::vector<Eigen::Vector3d> get_polygons();
  Eigen::Quaterniond get_quat_target();
  Eigen::Vector3d get_pos_target();
  void CC_vrpn_target(const geometry_msgs::PoseStamped::ConstPtr msg);
  void see_target();

private:
  bool targetReceived = false;
  double height_target, width_target;
  Eigen::Quaterniond targetQuat;
  Eigen::Vector3d targetPos;
  std::vector<Eigen::Vector3d> polygons_positions;
  ros::Subscriber poseTargetSub;
  ros::Publisher originalPolygonPub;
};
