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

// this class manage the boustrophedon Server 


class BoustrophedonServer {
public:
    BoustrophedonServer(ros::NodeHandle& n);
    void initRosLaunch();
    void closeRosLaunch();
    void setOptimumRad(double rad);
    ros::Publisher polygon_pub;
    ros::Publisher path_pub;
    ros::Publisher start_pub;
    actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction> client;

private:
    ros::NodeHandle nh;
    double optimumRad;
    pid_t launchProcessId; // To store the PID of the roslaunch process
};


