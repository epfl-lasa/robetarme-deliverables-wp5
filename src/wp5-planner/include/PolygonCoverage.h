/**
 * @file PolygonCoverage.h
 * @brief Declaration of the PolygonCoverage class
 * @author [Tristan Bonato]
 * @date 2024-03-07
 */

#pragma once

// #include <actionlib/client/simple_action_client.h>
// #include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sys/types.h>

#include <eigen3/Eigen/Dense>
#include <vector>

/**
 * @brief Class for managing the Boustrophedon server.
 *
 * This class initializes and closes ROS launch, sets the optimum radius,
 * and provides publishers and an action client for communication.
 */
class PolygonCoverage {
public:
  /**
   * @brief Constructor for PolygonCoverage.
   *
   * @param n Reference to the ROS node handle.
   */
  PolygonCoverage(ros::NodeHandle& n);

  /**
   * @brief Initialize ROS launch.
   */
  void initRosLaunch();

  /**
   * @brief Close ROS launch.
   */
  void closeRosLaunch();

  /**
   * @brief Set the optimum radius.
   *
   * @param rad The optimum radius to be set.
   */
  void setOptimumRad(double rad);

  /**
   * @brief function for run polygon coverage planning.
   *
   * @param  start The starting point.
    * @param  goal The goal point.
   */
  void callStartService(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

  void callSetPolygonService(const std::vector<Eigen::Vector3d>& hull_points,
                             const std::vector<Eigen::Vector3d>& hole_points);

  void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

  void convertPoseArrayToPath(const geometry_msgs::PoseArray& pose_array);
  nav_msgs::Path getPathFromPolygonFlat();
  void seePolygonFlat(std::vector<Eigen::Vector3d>);

  void publishNavmsg(nav_msgs::Path);
  void writePathToFile(const nav_msgs::Path& path, const std::string& file_path);
  nav_msgs::Path convertFileToNavMsgsPath();
  bool pointCloudTransformer();
  std::vector<Eigen::Vector3d> rdp(const std::vector<Eigen::Vector3d>& points, double epsilon);
  std::vector<std::vector<double>> convertNavPathToVectorVector(const nav_msgs::Path& inputPath);
  std::vector<Eigen::Vector3d> getFlatPolygonFromTxt();
  void getPathFromFeatureSpaceToRealSpace();
  void convertPclToPolygon();
  void featureSpaceAlgorithm();

private:
  ros::NodeHandle nh_;            ///< ROS node handle.
  ros::Publisher PolygonFlatPub_; ///< ROS publisher for the original polygon.
  ros::Publisher pathPubFlat_;    ///< ROS publisher for paths flat.
  ros::Publisher pathPubFinal_;   ///< ROS publisher for paths final.
  ros::Subscriber posArraySub_;   ///< ROS publisher for starting the action.
  double optimumRad;              ///< Optimum radius.
  pid_t launchProcessId;          ///< PID of the roslaunch process.
  double wallDistance_;           ///< Wall distance.
  double lateralFov_;             ///< Lateral field of view.
  nav_msgs::Path path_;           ///< Path.

  double perpendicularDistance(const Eigen::Vector3d& point,
                               const Eigen::Vector3d& lineStart,
                               const Eigen::Vector3d& lineEnd);
};
