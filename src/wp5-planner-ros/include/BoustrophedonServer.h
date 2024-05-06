/**
 * @file BoustrophedonServer.h
 * @brief Declaration of the BoustrophedonServer class
 * @author [Tristan Bonato]
 */

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <ros/ros.h>
#include <sys/types.h>

/**
 * @brief Class for managing the Boustrophedon server.
 *
 * This class initializes and closes ROS launch, sets the optimum radius,
 * and provides publishers and an action client for communication.
 */
class BoustrophedonServer {
public:
  /**
   * @brief Constructor for BoustrophedonServer.
   *
   * @param n Reference to the ROS node handle.
   */
  BoustrophedonServer(ros::NodeHandle& n);

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

  ros::Publisher polygonPub; ///< ROS publisher for polygons.
  ros::Publisher pathPub; ///< ROS publisher for paths.
  ros::Publisher startPub; ///< ROS publisher for starting the action.
  actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client; ///< Action client.

private:
  ros::NodeHandle nh; ///< ROS node handle.
  double optimumRad; ///< Optimum radius.
  pid_t launchProcessId; ///< PID of the roslaunch process.
};
