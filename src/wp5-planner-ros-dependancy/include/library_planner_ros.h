#ifndef LIBRARY_PLANNER_ROS_H
#define LIBRARY_PLANNER_ROS_H

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

// This class create a path that navigate throug the target depending the polygons
class PathPlanner {
public:
    std::vector<double> firstPos;
    double sum_rad, optimum_radius;

    Eigen::Quaterniond targetQuat;
    Eigen::Vector3d targetPos;
    geometry_msgs::PoseStamped initialPose;
    geometry_msgs::PoseWithCovarianceStamped initialPoseMsg;
    
    PathPlanner(ros::NodeHandle& nh, Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions);
    geometry_msgs::PoseStamped get_initial_pos_ros_msg();
    std::vector<Eigen::Vector3d> get_planner_points();
    boustrophedon_msgs::PlanMowingPathGoal  ComputeGoal();
    int optimization_parameter();
    void publishInitialPose();
    nav_msgs::Path get_transformed_path(const nav_msgs::Path& originalPath);
    void see_target_flat();
    // void set_strategique_position();
    bool convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path);
    bool convertStripingPlanToVectorVector(const boustrophedon_msgs::StripingPlan& striping_plan)
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

// this class manage the boustrophedon Server 


class BoustrophedonServer {
public:
    BoustrophedonServer(ros::NodeHandle& n, double rad);
    void closeRosLaunch();
    ros::Publisher polygon_pub;
    ros::Publisher path_pub;
    ros::Publisher start_pub;
    actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client;

private:
    ros::NodeHandle nh;
    double optimumRad;
    pid_t launchProcessId; // To store the PID of the roslaunch process
};




#endif  // LIBRARY_PLANNER_ROS_H
