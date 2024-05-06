/**
 * @file TargetExtraction.h
 * @brief Declaration of the TargetExtraction class
 * @author [Author Name]
 * @date [Date]
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include <vector>

/**
 * @brief Class for extracting information about the target.
 *
 * This class extracts information about the target, including its polygonal shape and center pose.
 * It is designed to be replaced by a similar extraction method from a CAD model in future implementations.
 */
class TargetExtraction {
public:
  /**
   * @brief Constructor for TargetExtraction.
   *
   * @param nh Reference to the ROS node handle.
   */
  TargetExtraction(ros::NodeHandle& nh);

  /**
   * @brief Get the polygon vertices of the target.
   *
   * @return Vector of Eigen::Vector3d representing the vertices of the target polygon.
   */
  std::vector<Eigen::Vector3d> getPolygons();

  /**
   * @brief Get the quaternion representing the orientation of the target.
   *
   * @return Quaternion representing the orientation of the target.
   */
  Eigen::Quaterniond getQuatTarget();

  /**
   * @brief Get the position of the target.
   *
   * @return Vector representing the position of the target.
   */
  Eigen::Vector3d getPosTarget();

  /**
   * @brief Callback function for receiving the target pose.
   *
   * @param msg Pointer to the received geometry_msgs::PoseStamped message.
   */
  void CCVrpnTarget(const geometry_msgs::PoseStamped::ConstPtr msg);

  /**
   * @brief Method to visualize the target with ROS1.
   */
  void seeTarget();

private:
  bool targetReceived_ = false; ///< Flag indicating if the target pose has been received.
  double heightTarget_, widthTarget_; ///< Height and width of the target.
  Eigen::Quaterniond targetQuat_; ///< Quaternion representing the orientation of the target.
  Eigen::Vector3d targetPos_; ///< Vector representing the position of the target.
  std::vector<Eigen::Vector3d> polygonsPositions_; ///< Vertices of the target polygon.
  ros::Subscriber poseTargetSub_; ///< ROS subscriber for receiving the target pose.
  ros::Publisher originalPolygonPub_; ///< ROS publisher for publishing the original polygon.
};
