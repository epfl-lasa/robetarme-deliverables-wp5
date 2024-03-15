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
  void parameterInitialization();
  void setPath(std::vector<std::vector<double>> firstQuatPos);
  void setCartPose(std::pair<Eigen::Quaterniond, Eigen::Vector3d>);
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getLinearDsOnePosition(std::vector<double> desiredQuatPos);
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getDsQuatSpeed();
  void updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3d pose, Eigen::Vector3d poseTarget);
  void setLinearSpeed(double speed);
  void setLimitCycleSpeedConv(double angSpeed, double conv);
  void setLimitCycleRadius(double rad);
  void setToleranceNextPoint(double tol);
  void restartPath();
  std::vector<double> getFirstQuatPos();

private:
  double ConvergenceRateLC_ = 10;
  double CycleRadiusLC_ = 0.03;
  double CycleSpeedLC_ = 2.5 * 3.14;
  double fs_ = 100;
  double toleranceToNextPoint_ = 0.1;
  double linearVelExpected_ = 0.04;

  std::size_t iFollow_ = 0;

  std::vector<std::vector<double>> desiredPath_;
  std::vector<double> firstQuatPos_;
  std::vector<double> lastQuatPos_;
  Eigen::Vector3d desiredVel_;

  Eigen::Vector3d realPos_;
  Eigen::Quaterniond realQuat_;

  Eigen::Vector3d realPosOffset_;
  Eigen::Quaterniond realQuatOffset_;

  Eigen::Vector3d centerLimitCycle_;
  Eigen::Vector4d desiredOriVelocityFiltered_;

  double toolOffsetFromTarget_, velocityLimit_;
  bool targetReceived__ = false;
};
