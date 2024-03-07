#pragma once


// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>  
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include <thread>   // Include this header for std::this_thread
#include <chrono>   // Include this header for std::chrono


//This class managed the Dynamical system to return the desired veloctiy  in function of the eef pose and path
class DynamicalSystem {
public:
    bool finish =false;

    DynamicalSystem( double freq);
    void parameter_initialization();
    void set_path(std::vector<std::vector<double>> firstQuatPos);
    void addOffsetEef(Eigen::Vector3d pos, Eigen::Vector4d quat) ;
    Eigen::Vector3d get_DS_vel();
    Eigen::Matrix3d quaternionToRotationMatrix(Eigen::Vector4d q);
    void updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3d pose, Eigen::Vector3d target_pose_cricleDS);
    void set_linear_speed(double speed);
    void set_limitCycle_speed_conv(double angSpeed,double conv);
    void set_limitCycle_radius(double rad);
    void set_tolerance_next_point(double tol);
    void restart_path();

private:
    double ConvergenceRateLC    = 10;
    double CycleRadiusLC        = 0.03;
    double CycleSpeedLC         = 2.5* 3.14;
    double fs                   = 100;
    double toleranceToNextPoint = 0.1;
    double linearVelExpected    = 0.04;

    std::size_t iFollow = 0;

    std::vector<std::vector<double>> desiredPath;
    std::vector<double> firstQuatPos;
    std::vector<double> lastQuatPos;
    Eigen::Vector3d desiredVel;

    Eigen::Vector3d realPos;
    Eigen::Quaterniond realQuat;	
        
    Eigen::Vector3d realPosOffset;
    Eigen::Quaterniond realQuatOffset;

    Eigen::Vector3d centerLimitCycle;
    Eigen::Vector4d desiredOriVelocityFiltered_;
    std::string robot_name;

    double toolOffsetFromTarget, velocityLimit;
    bool targetReceived = false;
    std::vector<Eigen::Vector3d> polygons_positions;

};


