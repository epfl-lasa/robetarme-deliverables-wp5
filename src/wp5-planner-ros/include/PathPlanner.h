#pragma once

#include <boustrophedon_msgs/ConvertPlanToPath.h>
#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <vector>

// This class create a path that navigate throug the target depending the polygons
class PathPlanner {
public:
  std::vector<double> firstPos;
  double sumRad, optimumRadius;

  Eigen::Quaterniond targetQuat;
  Eigen::Vector3d targetPos;
  geometry_msgs::PoseStamped initialPose;
  geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;

  PathPlanner(ros::NodeHandle& n);
  void setTarget(Eigen::Quaterniond targetOrientation,
                 Eigen::Vector3d targetPosition,
                 std::vector<Eigen::Vector3d> polygonsPos);
  double getOptimumRadius();
  geometry_msgs::PoseStamped getInitialPose();
  geometry_msgs::PoseStamped getInitialPosRosMsg();
  std::vector<Eigen::Vector3d> getPlannerPoints();
  boustrophedon_msgs::PlanMowingPathGoal ComputeGoal();
  int optimization_parameter();
  void publishInitialPose();
  nav_msgs::Path getPlannerPoints(const nav_msgs::Path& originalPath);
  void seeTargetFlat();
  nav_msgs::Path getTransformedPath(const nav_msgs::Path& originalPath);
  bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& stripingPlan, nav_msgs::Path& path);
  std::vector<std::vector<double>> convertPathPlanToVectorVector(const nav_msgs::Path& path);
  geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);

private:
  double flowRadius_, limitCycleRadius_, scaleFactor_;
  ros::NodeHandle nh_;
  ros::Publisher initialPosePub_;
  ros::Publisher transformedPolygonPub_;
  std::vector<Eigen::Vector3d> polygonsPositions_;
  std::vector<Eigen::Vector3d> flatPolygons_;
  Eigen::Vector3d findCenter(const std::vector<Eigen::Vector3d>& vertices);
  void scalePolygon(std::vector<Eigen::Vector3d>& vertices);
  double find_height();
};
