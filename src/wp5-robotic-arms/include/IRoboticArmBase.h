/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "dynamical_systems/DynamicalSystemFactory.hpp"
// clang-format off
#include "pinocchio/fwd.hpp"
// clang-format on
#include "robot_model/Model.hpp"
#include "state_representation/space/cartesian/CartesianPose.hpp"
#include "state_representation/space/cartesian/CartesianTwist.hpp"
#include "state_representation/space/joint/JointPositions.hpp"
#include "state_representation/space/joint/JointVelocities.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <trac_ik/trac_ik.hpp>
#include <vector>

using namespace std;

/**
 * @brief Mother class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
class IRoboticArmBase {
  /* TODO: Implement all the common functions for all the robotic arms, make them pure virtual if the
   function is not implemented in the mother class */
public:
  // TODO: implement all the public members, accessible from everyone owning a class object
  explicit IRoboticArmBase() = default;
  int numberJoint = 0;
  string reference_frame = "";
  vector<double> getFK(vector<double>);
  geometry_msgs::Twist getTwist(vector<double>, vector<double>);
  MatrixXd getJacobian(vector<double>);
  pair<int, vector<double>> getIK(vector<double> , vector<double> ) ;
  void updateIK(double , double );

  virtual void low_level_controller();

protected:
  // TODO: implement all the protected members, accessible from its own and herited classes

  string robot_name = "";
  vector<string> controller_joint_names = "";
  string baseLink ="";
  string tipLink = "";
  string tipJoint = "";
  string path_urdf = "";
  unique_ptr<robot_model::Model> model;

  string URDF_param="";
  int  nJoint = 0  ;


  vector<double> posJointNext = {};
  int rc = 0;
  TRAC_IK::TRAC_IK* ikSolver = nullptr;  
  TRAC_IK::SolveType type =TRAC_IK::Distance;
  KDL::Chain chain = {};
  bool valid = false ;


  // TODO put in YAML
  double error = 0.01;
  double timeoutInSecs = 0.5;

private:
  // TODO: implement all the private members, only accessible from its own class
};
