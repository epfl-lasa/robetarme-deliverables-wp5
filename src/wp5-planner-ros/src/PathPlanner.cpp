#include "PathPlanner.h"

#include <cstdlib>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <ros/package.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

// Constructor definition
PathPlanner::PathPlanner(ros::NodeHandle& n) :
    initialPosePub_(nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10)) {
  nh_ = n;
  transformedPolygonPub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/flat_polygon", 1, true);
  string package_path = ros::package::getPath("wp5-planner-ros");
  // Load parameters from YAML file
  string yaml_path = package_path + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);

  // Access parameters from the YAML file
  limitCycleRadius_ = config["limitCycleRadius"].as<double>();
  flowRadius_ = config["flowRadius"].as<double>();
  sumRad = flowRadius_ + limitCycleRadius_;
}

void PathPlanner::setTarget(Quaterniond targetOrientation, Vector3d targetPosition, vector<Vector3d> polygonsPos) {
  targetQuat = targetOrientation;
  targetPos = targetPosition;
  polygonsPositions_ = polygonsPos;
  optimization_parameter();
}

double PathPlanner::getOptimumRadius() { return optimumRadius; }

geometry_msgs::PoseStamped PathPlanner::getInitialPose() { return initialPose; }

// Function to find the center of the polygon
Vector3d PathPlanner::findCenter(const vector<Vector3d>& vertices) {
  Vector3d center(0.0, 0.0, 0.0);
  for (const auto& vertex : vertices) {
    center += vertex;
  }
  center /= static_cast<double>(vertices.size());
  return center;
}

// Function to scale a polygon around its center
void PathPlanner::scalePolygon(vector<Vector3d>& vertices) {

  // Find the center of the polygon
  Vector3d center = findCenter(vertices);
  //calculate the sacelfactor
  Vector3d diff = center - vertices[0];
  double d = diff.norm();
  double scaleFactor_ = (d - optimumRadius * 0.8) / d;

  // Translate the polygon to the origin
  for (auto& vertex : vertices) {
    vertex -= center;
  }

  // Apply scaling factor to the vertices
  for (auto& vertex : vertices) {
    vertex *= scaleFactor_;
  }

  // Translate the polygon back to its original position
  for (auto& vertex : vertices) {
    vertex += center;
  }
}

vector<Vector3d> PathPlanner::getPlannerPoints() {

  vector<Vector3d> rotated_points;

  PathPlanner::scalePolygon(polygonsPositions_);

  Affine3d transformation = Translation3d(targetPos(0), targetPos(1), targetPos(2)) * targetQuat.conjugate();

  MatrixXd points_matrix(polygonsPositions_.size(), 3);
  for (size_t i = 0; i < polygonsPositions_.size(); ++i) {
    Vector3d rotated_point = transformation.inverse() * polygonsPositions_[i];
    rotated_points.push_back(rotated_point);
  }
  return rotated_points;
}

boustrophedon_msgs::PlanMowingPathGoal PathPlanner::ComputeGoal() {
  flatPolygons_ = getPlannerPoints();

  Vector3d p1 = flatPolygons_[0];
  Vector3d p2 = flatPolygons_[1];
  Vector3d p3 = flatPolygons_[2];
  Vector3d p4 = flatPolygons_[3];

  //polygon for the server
  boustrophedon_msgs::PlanMowingPathGoal goal;

  goal.property.header.stamp = ros::Time::now();
  goal.property.header.frame_id = "base";
  goal.property.polygon.points.resize(4);

  goal.property.polygon.points[0].x = p1(0);
  goal.property.polygon.points[0].y = p1(1);
  goal.property.polygon.points[0].z = p1(2);

  goal.property.polygon.points[1].x = p2(0);
  goal.property.polygon.points[1].y = p2(1);
  goal.property.polygon.points[1].z = p2(2);

  goal.property.polygon.points[2].x = p3(0);
  goal.property.polygon.points[2].y = p3(1);
  goal.property.polygon.points[2].z = p3(2);

  goal.property.polygon.points[3].x = p4(0);
  goal.property.polygon.points[3].y = p4(1);
  goal.property.polygon.points[3].z = p4(2);

  goal.robot_position.pose.orientation.x = 0.0;
  goal.robot_position.pose.orientation.y = 0.0;
  goal.robot_position.pose.orientation.z = 0.0;
  goal.robot_position.pose.orientation.w = 1.0;

  return goal;
}

double PathPlanner::find_height() {
  if (polygonsPositions_.empty()) {
    // Handle the case when there are no points
    return 0.0;
  }

  double maxZ = -numeric_limits<double>::infinity();
  double minZ = numeric_limits<double>::infinity();

  vector<Vector3d> lowestZPoints;
  vector<Vector3d> highestZPoints;

  for (const auto& point : polygonsPositions_) {
    if (point.z() > maxZ) {
      maxZ = point.z();
      highestZPoints.clear();
      highestZPoints.push_back(point);
    } else if (point.z() < minZ) {
      minZ = point.z();
      lowestZPoints.clear();
      lowestZPoints.push_back(point);
    }
  }

  if (lowestZPoints.empty() || highestZPoints.empty()) {
    // Handle the case when vectors are empty (no points found)
    return 0.0;
  }

  Vector3d diff = lowestZPoints[0] - highestZPoints[0];
  double height = diff.norm();
  return height;
}

int PathPlanner::optimization_parameter() {
  double D = find_height();

  if (D == 0.0) {
    optimumRadius = sumRad;
    return 0;
  }
  double d = sumRad;
  double n = D / d;
  int roundedNumber = round(n) + 4;
  double r_new = D / roundedNumber;
  optimumRadius = r_new;
  return 1;
}

void PathPlanner::publishInitialPose() {
  double maxZ = -numeric_limits<double>::infinity();
  vector<Vector3d> highestZPoints;
  int imax = 0;
  int i = 0;

  vector<Vector3d> points = polygonsPositions_;
  for (const auto& point : points) {
    if (point.z() > maxZ) {
      maxZ = point.z();
      highestZPoints.clear(); // Clear previous points with lower z
      highestZPoints.push_back(point);
      imax = i;
    }
    i++;
  }
  vector<Vector3d> flatPolygons_ = getPlannerPoints();
  Vector3d pointInitial = flatPolygons_[imax];
  seeTargetFlat();
  cout << points[imax] << endl;
  double delta = 0.1;
  // Create a publisher for the /initialpose topic

  // Create and fill the message
  initialPoseMsg.header.seq = 0;
  initialPoseMsg.header.stamp = ros::Time(0);
  initialPoseMsg.header.frame_id = "base";

  initialPoseMsg.pose.pose.position.x = pointInitial(0) + delta;
  initialPoseMsg.pose.pose.position.y = pointInitial(1) - delta;
  initialPoseMsg.pose.pose.position.z = pointInitial(2);

  initialPoseMsg.pose.pose.orientation.x = 0.0;
  initialPoseMsg.pose.pose.orientation.y = 0.0;
  initialPoseMsg.pose.pose.orientation.z = 0.0;
  initialPoseMsg.pose.pose.orientation.w = 1.0;

  initialPoseMsg.pose.covariance.fill(0.0); // Fill the covariance with zeros

  initialPosePub_.publish(initialPoseMsg);
  initialPose.header = initialPoseMsg.header;
  initialPose.pose = initialPoseMsg.pose.pose;
}

nav_msgs::Path PathPlanner::getTransformedPath(const nav_msgs::Path& originalPath) {
  nav_msgs::Path transformedPath;
  Affine3d transformation = Translation3d(targetPos(0), targetPos(1), targetPos(2)) * targetQuat.conjugate();
  ;

  transformedPath.header = originalPath.header;

  for (const auto& pose : originalPath.poses) {
    // Convert pose to Eigen types for transformation
    Vector3d originalPosition(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    Quaterniond originalOrientation(pose.pose.orientation.w,
                                    pose.pose.orientation.x,
                                    pose.pose.orientation.y,
                                    pose.pose.orientation.z);

    // Apply transformation
    Vector3d transformedPosition = transformation * originalPosition;
    Quaterniond transformedOrientation = targetQuat;

    // Convert back to geometry_msgs types
    geometry_msgs::PoseStamped transformedPose;
    transformedPose.header = originalPath.header;
    transformedPose.pose.position.x = transformedPosition.x();
    transformedPose.pose.position.y = transformedPosition.y();
    transformedPose.pose.position.z = transformedPosition.z();
    transformedPose.pose.orientation.w = transformedOrientation.w();
    transformedPose.pose.orientation.x = transformedOrientation.x();
    transformedPose.pose.orientation.y = transformedOrientation.y();
    transformedPose.pose.orientation.z = transformedOrientation.z();

    // Add the transformed pose to the new path
    transformedPath.poses.push_back(transformedPose);
  }
  geometry_msgs::PoseStamped first_pose = transformedPath.poses[0];

  // Extract position (x, y, z)
  double x = first_pose.pose.position.x;
  double y = first_pose.pose.position.y;
  double z = first_pose.pose.position.z;
  firstPos = {x, y, z};
  // set_strategique_position();
  return transformedPath;
}

void PathPlanner::seeTargetFlat() {
  geometry_msgs::PolygonStamped visualpolygonTarget;
  visualpolygonTarget.header.frame_id = "base";
  visualpolygonTarget.header.stamp = ros::Time::now();

  for (const auto& point : flatPolygons_) {
    geometry_msgs::Point32 msg_point;
    msg_point.x = point.x();
    msg_point.y = point.y();
    msg_point.z = point.z();
    visualpolygonTarget.polygon.points.push_back(msg_point);
  }
  transformedPolygonPub_.publish(visualpolygonTarget);
}

// server has a service to convert StripingPlan to Path, but all it does it call this method
bool PathPlanner::convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& stripingPlan,
                                            nav_msgs::Path& path) {
  path.header.frame_id = stripingPlan.header.frame_id;
  path.header.stamp = stripingPlan.header.stamp;

  path.poses.clear();
  for (size_t i = 0; i < stripingPlan.points.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = stripingPlan.header.frame_id;
    pose.header.stamp = stripingPlan.header.stamp;
    pose.pose.position = stripingPlan.points[i].point;

    if (i < stripingPlan.points.size() - 1) {
      double dx = stripingPlan.points[i + 1].point.x - stripingPlan.points[i].point.x;
      double dy = stripingPlan.points[i + 1].point.y - stripingPlan.points[i].point.y;
      double dz = stripingPlan.points[i + 1].point.z - stripingPlan.points[i].point.z;

      pose.pose.orientation = headingToQuaternion(dx, dy, dz);
    } else {
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
    }

    path.poses.push_back(pose);
  }

  return true;
}
vector<vector<double>> PathPlanner::convertPathPlanToVectorVector(const nav_msgs::Path& inputPath) {

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

geometry_msgs::Quaternion PathPlanner::headingToQuaternion(double x, double y, double z) {
  // get orientation from heading vector
  const tf2::Vector3 headingVector(x, y, z);
  const tf2::Vector3 origin(1, 0, 0);

  const auto w = (origin.length() * headingVector.length()) + tf2::tf2Dot(origin, headingVector);
  const tf2::Vector3 a = tf2::tf2Cross(origin, headingVector);
  tf2::Quaternion q(a.x(), a.y(), a.z(), w);
  q.normalize();

  if (!isfinite(q.x()) || !isfinite(q.y()) || !isfinite(q.z()) || !isfinite(q.w())) {
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    q.setW(1);
  }

  return tf2::toMsg(q);
}
