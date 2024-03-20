/**
 * @file IRoboticArmBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include "IRoboticArmBase.h"

#include <OsqpEigen/OsqpEigen.h>
#include <yaml-cpp/yaml.h>

#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>

using namespace std;
using namespace Eigen;

/**
 * @brief Mother class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */
pair<Quaterniond, Vector3d> IRoboticArmBase::getFK(vector<double> vectJoint) {
  Map<VectorXd> posJointEigen(vectJoint.data(), vectJoint.size());
  state_representation::JointPositions nextJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);
  state_representation::CartesianPose nextCartesianPose = model_->forward_kinematics(nextJoinState, tipLink_);

  Vector3d p1Prime = nextCartesianPose.get_position();
  Quaterniond q1Prime = nextCartesianPose.get_orientation();

  return make_pair(q1Prime, p1Prime);
}

VectorXd IRoboticArmBase::getTwistFromJointState(vector<double> posJoint, vector<double> speedJoint) {
  MatrixXd jacMatrix = getJacobian(posJoint);
  VectorXd speedJointEigen(nJoint_);

  for (int i = 0; i < nJoint_; ++i) {
    speedJointEigen(i) = speedJoint[i];
  }

  VectorXd twistLinearAngular = jacMatrix * speedJointEigen;

  return twistLinearAngular;
}

MatrixXd IRoboticArmBase::getJacobian(vector<double> vectJoint) {
  Map<VectorXd> posJointEigen(vectJoint.data(), vectJoint.size());
  state_representation::JointPositions actualJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);
  state_representation::Jacobian jacobianObject = model_->compute_jacobian(actualJoinState);
  MatrixXd jacobian = jacobianObject.data();

  return jacobian;
}
vector<double> IRoboticArmBase::getIKinematics(vector<double> vectJoint,
                                               const pair<Quaterniond, Vector3d> pairQuatPos) {
  Quaterniond orientation = pairQuatPos.first;
  Vector3d position = pairQuatPos.second;
  vector<double> positionJointNext(nJoint_);

  Map<VectorXd> posJointEigen(vectJoint.data(), vectJoint.size());
  state_representation::JointPositions actualJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);

  state_representation::CartesianPose cartesianPose =
      state_representation::CartesianPose(robotName_, position, orientation, referenceFrame_);

  state_representation::JointPositions nextJoinState =
      model_->inverse_kinematics(cartesianPose, actualJoinState, paramsIK_, referenceFrame_);

  VectorXd positionJointNext_eigen = nextJoinState.data();

  for (int i = 0; i < nJoint_; ++i) {
    positionJointNext[i] = positionJointNext_eigen(i);
  }

  return positionJointNext;
}

vector<double> IRoboticArmBase::getInvertVelocities(vector<double> vectJoint, VectorXd speedEigen) {
  vector<double> speedJointNext(nJoint_);

  Vector3d angularVelocity(3);
  Vector3d linearVelocity(3);
  linearVelocity << speedEigen(0), speedEigen(1), speedEigen(2);
  angularVelocity << speedEigen(3), speedEigen(4), speedEigen(5);

  Map<VectorXd> posJointEigen(vectJoint.data(), vectJoint.size());

  state_representation::JointPositions actualJoinState =
      state_representation::JointPositions(robotName_, jointNames_, posJointEigen);
  state_representation::CartesianTwist nextPostwist =
      state_representation::CartesianTwist(robotName_, linearVelocity, angularVelocity, referenceFrame_);
  state_representation::JointVelocities nextJoinStateSpeed =
      model_->inverse_velocity(nextPostwist, actualJoinState, tipJoint_);
  VectorXd speedJointNext_eigen = nextJoinStateSpeed.data();
  for (int i = 0; i < nJoint_; ++i) {
    speedJointNext[i] = speedJointNext_eigen(i);
  }

  return speedJointNext;
}


Eigen::VectorXd IRoboticArmBase::transformWrenchToBase(const Eigen::VectorXd& wrench_end_effector,
                                                       const Eigen::Vector3d& position_end_effector,
                                                       const Eigen::Quaterniond& orientation_end_effector_to_base) {
  // Extract rotation quaternion from end effector to base transformation
  Eigen::Matrix3d rotation_matrix = orientation_end_effector_to_base.toRotationMatrix();
  // Extract force and torque from wrench
  Eigen::Vector3d force_end_effector = wrench_end_effector.head<3>();
  Eigen::Vector3d torque_end_effector = wrench_end_effector.tail<3>();
  // Transform wrench to the base frame
  Eigen::Vector3d force_base = rotation_matrix * force_end_effector;
  Eigen::Vector3d torque_base = rotation_matrix * torque_end_effector;
  // Combine transformed force and torque into a single vector
  Eigen::VectorXd wrench_base(6);
  wrench_base << force_base(0), force_base(1), force_base(2), torque_base(0), torque_base(1), torque_base(2);
  return wrench_base;
}
// function for the inverse kinematic  ------------------------------------------------------------------------

// pair<int, vector<double>> IRoboticArmBase::getIK(vector<double> actualJoint, vector<double> vectorQuatPos ) {

//   //Inverse kinematics trac-IK
//   KDL::JntArray NextJointTask;
//   KDL::JntArray actualJointTask;

//   VectorXd pos_joint_actual_eigen(nJoint_);
//   for(int i = 0 ;i<nJoint;++i){
//       pos_joint_actual_eigen(i) =actualJoint[i];
//   }
//   actualJointTask.data = pos_joint_actual_eigen;

//   KDL::Vector Vec(vectorQuatPos[4],vectorQuatPos[5],vectorQuatPos[6]);

//   Quaterniond q(vectorQuatPos[3],vectorQuatPos[0],vectorQuatPos[1],vectorQuatPos[2]);
//   q.normalize();
//   KDL::Rotation Rot = KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w());
//   KDL::Frame NextJointCartesian(Rot,Vec);
//   rc = ikSolver->CartToJnt(actualJointTask, NextJointCartesian, NextJointTask);
//   if (rc< 0){
//       cout<<"no inverse kinematic found"<<endl;
//   }

//   VectorXd posJointNextEigen = NextJointTask.data;
//   vector<double> posJointNext ;
//   for(int i = 0 ;i<nJoint;++i){
//       posJointNext[i] =posJointNextEigen(i);
//       }
//   //msgP.data = posJointNext;

//   //actualJointTask.data.clear();
//   pair<int, vector<double>> myPair = make_pair(rc, posJointNext);

//   return myPair;
// }

// void IRoboticArmBase::updateIK(double err ,double timeoutInSecs, string solveTypeStr ){
//   // Convert the solve type string to the corresponding enum value
//   TRAC_IK::SolveType solveType;
//   if (solveTypeStr == "Distance") {
//       solveType = TRAC_IK::Distance;
//   } else if (solveTypeStr == "Speed") {
//       solveType = TRAC_IK::Speed;
//   } else {
//       cout<< "Handle unrecognized solve types: set Distance as default value" << endl;
//       solveType = TRAC_IK::Distance;
//   }
//   ikSolver= new TRAC_IK::TRAC_IK(baseLink_, tipLink_,paramURDF_, timeoutInSecs, err, solveType);
// }

// void IRoboticArmBase::initIK(){
//   YAML::Node config = YAML::LoadFile("/../config/config.yaml");
//   // Get the solve type from the YAML file
//   string solveTypeStr = config["IK/solve_type"].as<string>();

//   // Convert the solve type string to the corresponding enum value
//   TRAC_IK::SolveType solveType;
//   if (solveTypeStr == "Distance") {
//       solveType = TRAC_IK::Distance;
//   } else if (solveTypeStr == "Speed") {
//       solveType = TRAC_IK::Speed;
//   } else {
//       cout<< "Handle unrecognized solve types: set Distance as default value" << endl;
//       solveType = TRAC_IK::Distance;
//   }
//   error = config["IK/error"].as<double>();
//   timeoutInSecs = config["IK/timeoutInSecs"].as<double>();

//   ikSolver= new TRAC_IK::TRAC_IK(baseLink_, tipLink_,paramURDF_, timeoutInSecs, error, type);

//   valid = ikSolver->getKDLChain(chain);
//   if (!valid) {
//       cout << "There was no valid KDL chain found"<< endl;
//   }
// }
