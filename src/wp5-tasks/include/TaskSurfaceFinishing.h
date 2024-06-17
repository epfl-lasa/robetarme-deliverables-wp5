#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <ros/ros.h>

#include <memory>
#include <string>

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "ITaskBase.h"
#include "PathPlanner.h"
#include "PolygonCoverage.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"

class TaskSurfaceFinishing : public ITaskBase {
public:
  TaskSurfaceFinishing(ros::NodeHandle& n, double freq, std::string robotName);

  bool computePath();
  bool execute();

  void set_bias();

  std::vector<double> homeJoint_;
  std::vector<double> biasWrench_;
  Eigen::VectorXd outputTwist_;

private:
  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetExtraction_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathPlanner_ = nullptr;

  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<BoustrophedonServer> boustrophedonServer_ = nullptr;

  std::unique_ptr<PolygonCoverage> polygonCoverage_ = nullptr;

  Eigen::VectorXd decoderWrench();
  Eigen::VectorXd contactUpdateDS();
  double makeContact();
  double PIController(double error);

  double desiredContactForce_;
  double twistForContactForce_;
  std::vector<std::vector<double>> vectorPathTransformed_;
  double kp_;           // Proportional gain
  double ki_;           // Integral gain
  double integral_;     // Integral term
  ros::Time prev_time_; // Previous time for integral calculation
};
