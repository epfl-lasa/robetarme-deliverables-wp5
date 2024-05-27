/**
 * @file PathPlanner.h
 * @brief Declaration of the PathPlanner class
 * @author [Tristan Bonato]
 * @date 2024-03-07
 */

#pragma once

#include <boustrophedon_msgs/ConvertPlanToPath.h>
#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <vector>

/**
 * @brief Class for path planning based on target polygons.
 *
 * This class creates a path that navigates through the target based on given polygons.
 */
class PathPlanner {
public:
  std::vector<double> firstPos; ///< Initial position.
  double sumRad, optimumRadius; ///< Sum of radii and optimum radius.

  Eigen::Quaterniond targetQuat;                           ///< Quaternion representing target orientation.
  Eigen::Vector3d targetPos;                               ///< Vector representing target position.
  geometry_msgs::PoseStamped initialPose;                  ///< Initial pose.
  geometry_msgs::PoseWithCovarianceStamped initialPoseMsg; ///< Initial pose message.

  /**
   * @brief Constructor for PathPlanner.
   *
   * @param n Reference to the ROS node handle.
   */
  PathPlanner(ros::NodeHandle& n);

  /**
   * @brief Set the target information.
   *
   * @param targetOrientation Quaternion representing the target orientation.
   * @param targetPosition Vector representing the target position.
   * @param polygonsPos Vector of vectors representing the positions of polygons.
   */
  void setTarget(Eigen::Quaterniond targetOrientation,
                 Eigen::Vector3d targetPosition,
                 std::vector<Eigen::Vector3d> polygonsPos);

  /**
   * @brief Get the optimum radius.
   *
   * @return Optimum radius.
   */
  double getOptimumRadius();

  /**
   * @brief Get the initial pose.
   *
   * @return Initial pose.
   */
  geometry_msgs::PoseStamped getInitialPose();

  /**
   * @brief Get the initial pose as a ROS message.
   *
   * @return Initial pose as a ROS message.
   */
  geometry_msgs::PoseStamped getInitialPosRosMsg();

  /**
   * @brief Get the planner points.
   *
   * @return Vector of vectors representing planner points.
   */
  std::vector<Eigen::Vector3d> getPlannerPoints();

  /**
   * @brief Compute the goal for mowing path.
   *
   * @return Goal for mowing path.
   */
  boustrophedon_msgs::PlanMowingPathGoal ComputeGoal();

  /**
   * @brief Perform optimization parameter calculation.
   *
   * @return Result of optimization parameter calculation.
   */
  int optimization_parameter();

  /**
   * @brief Publish the initial pose.
   */
  void publishInitialPose();
  void publishInitialPose(Eigen::Vector3d pointInitial);

  /**
   * @brief Get the transformed path.
   *
   * @param originalPath Original path to be transformed.
   * @return Transformed path.
   */
  nav_msgs::Path getTransformedPath(const nav_msgs::Path& originalPath);

  /**
   * @brief Convert striping plan to path.
   *
   * @param stripingPlan Striping plan to be converted.
   * @param path Path to store the result.
   * @return True if conversion is successful, otherwise false.
   */
  bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& stripingPlan, nav_msgs::Path& path);

  /**
   * @brief Convert path plan to vector of vectors.
   *
   * @param path Path to be converted.
   * @return Vector of vectors representing the path plan.
   */
  std::vector<std::vector<double>> convertPathPlanToVectorVector(const nav_msgs::Path& path);

  /**
   * @brief Convert heading to quaternion.
   *
   * @param x X-coordinate.
   * @param y Y-coordinate.
   * @param z Z-coordinate.
   * @return Quaternion representing the heading.
   */
  geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);

  /**
   * @brief Method to visualize the flat target.
   */
  void seeTargetFlat();

private:
  double flowRadius_, limitCycleRadius_, scaleFactor_; ///< Flow radius, limit cycle radius, and scale factor.
  ros::NodeHandle nh_;                                 ///< ROS node handle.
  ros::Publisher initialPosePub_;                      ///< ROS publisher for the initial pose.
  ros::Publisher transformedPolygonPub_;               ///< ROS publisher for the transformed polygon.
  std::vector<Eigen::Vector3d> polygonsPositions_;     ///< Positions of polygons.
  std::vector<Eigen::Vector3d> flatPolygons_;          ///< Flat polygons.

  /**
   * @brief Find the center of a polygon.
   *
   * @param vertices Vertices of the polygon.
   * @return Center of the polygon.
   */
  Eigen::Vector3d findCenter(const std::vector<Eigen::Vector3d>& vertices);

  /**
   * @brief Scale the polygon.
   *
   * @param vertices Vertices of the polygon.
   */
  void scalePolygon(std::vector<Eigen::Vector3d>& vertices);

  /**
   * @brief Find the height of the target.
   *
   * @return Height of the target.
   */
  double find_height();
};
