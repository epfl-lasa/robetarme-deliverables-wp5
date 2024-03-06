#pragma once


#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>  
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <boustrophedon_msgs/PlanMowingPathAction.h>
#include <boustrophedon_msgs/ConvertPlanToPath.h>
#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include <thread>   // Include this header for std::this_thread
#include <chrono>   // Include this header for std::chrono


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

