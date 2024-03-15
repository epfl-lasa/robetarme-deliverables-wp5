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
  std::vector<Eigen::Vector3d> getPolygons();
  Eigen::Quaterniond getQuatTarget();
  Eigen::Vector3d getPosTarget();
  void CCVrpnTarget(const geometry_msgs::PoseStamped::ConstPtr msg);
  void seeTarget();

private:
  bool targetReceived_ = false;
  double heightTarget_, widthTarget_;
  Eigen::Quaterniond targetQuat_;
  Eigen::Vector3d targetPos_;
  std::vector<Eigen::Vector3d> polygonsPositions_;
  ros::Subscriber poseTargetSub_;
  ros::Publisher originalPolygonPub_;
};
