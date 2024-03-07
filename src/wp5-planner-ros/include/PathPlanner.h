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


// This class create a path that navigate throug the target depending the polygons
class PathPlanner {
public:
    std::vector<double> firstPos;
    double sum_rad, optimum_radius;

    Eigen::Quaterniond targetQuat;
    Eigen::Vector3d targetPos;
    geometry_msgs::PoseStamped initialPose;
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    
    // PathPlanner(ros::NodeHandle& nh, Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions);
    PathPlanner(ros::NodeHandle& nh);
    void setTarget( Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions);
    double getOptimumRadius();
    geometry_msgs::PoseStamped getInitialPose();
    geometry_msgs::PoseStamped get_initial_pos_ros_msg();
    std::vector<Eigen::Vector3d> get_planner_points();
    boustrophedon_msgs::PlanMowingPathGoal  ComputeGoal();
    int optimization_parameter();
    void publishInitialPose();
    nav_msgs::Path get_transformed_path(const nav_msgs::Path& originalPath);
    void see_target_flat();
    // void set_strategique_position();
    bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path);
    bool convertPathPlanToVectorVector(const  nav_msgs::Path& path);
    geometry_msgs::Quaternion headingToQuaternion(double x, double y, double z);

private:
    double flow_radius, limit_cycle_radius, toolOffsetFromTarget, scaleFactor;
    ros::NodeHandle nh;
    ros::Publisher initialPosePub_;
    ros::Publisher transformedPolygonPub;
    std::vector<Eigen::Vector3d> polygonsPositions;
    std::vector<Eigen::Vector3d> flatPolygons;
    Eigen::Vector3d findCenter(const std::vector<Eigen::Vector3d>& vertices);
    void scalePolygon(std::vector<Eigen::Vector3d>& vertices);
    double  find_height();
};


