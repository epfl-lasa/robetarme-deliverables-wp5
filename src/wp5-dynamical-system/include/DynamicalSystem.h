#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

//This class managed the Dynamical system to return the desired veloctiy  in function of the eef pose and path
class DynamicalSystem {
public:
  bool finish = false;
  bool init = false;
  bool checkLinearDs = false;
  Eigen::Vector3d pathPoint;
  DynamicalSystem(double freq);
  void parameter_initialization();
  void set_path(std::vector<std::vector<double>> firstQuatPos);
  void setCartPose(std::pair<Eigen::Quaterniond, Eigen::Vector3d>);
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getLinearDsOnePosition(std::vector<double> desiredQuatPos);
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> get_DS_quat_speed();
  void updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3d pose, Eigen::Vector3d target_pose_cricleDS);
  void set_linear_speed(double speed);
  void set_limitCycle_speed_conv(double angSpeed, double conv);
  void set_limitCycle_radius(double rad);
  void set_tolerance_next_point(double tol);
  void restart_path();
  std::vector<double> getFirstQuatPos();

private:
  double ConvergenceRateLC = 10;
  double CycleRadiusLC = 0.03;
  double CycleSpeedLC = 2.5 * 3.14;
  double fs = 100;
  double toleranceToNextPoint = 0.1;
  double linearVelExpected = 0.04;

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
