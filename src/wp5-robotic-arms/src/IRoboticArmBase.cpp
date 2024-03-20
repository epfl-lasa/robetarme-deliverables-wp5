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

VectorXd IRoboticArmBase::getTwistFromDS(Quaterniond quat1, pair<Quaterniond, Vector3d> pairQuatPos) {
  Quaterniond quat2 = pairQuatPos.first;
  Vector3d speed = pairQuatPos.second;
  //orientation
  Vector4d q1, q2;
  q1 << quat1.w(), quat1.x(), quat1.y(), quat1.z(); //qw,qx,qy,qz
  q2 << quat2.w(), quat2.x(), quat2.y(), quat2.z(); //qw,qx,qy,qz

  Vector4d dqd = slerpQuaternion(q1, q2, 0.5);
  Vector4d deltaQ = dqd - q1;

  Vector4d qconj = q1;
  qconj.segment(1, 3) = -1 * qconj.segment(1, 3);
  Vector4d temp_angVel = quaternionProduct(deltaQ, qconj);

  Vector3d tmp_angular_vel = temp_angVel.segment(1, 3);
  double maxDq = 0.2;
  if (tmp_angular_vel.norm() > maxDq)
    tmp_angular_vel = maxDq * tmp_angular_vel.normalized();

  double dsGain_ori = 0.50;
  double theta_gq = (-.5 / (4 * maxDq * maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
  Vector3d Omega_out = 2 * dsGain_ori * (1 + exp(theta_gq)) * tmp_angular_vel;

  vector<double> V = {speed(0), speed(1), speed(2), Omega_out[0], Omega_out[1], Omega_out[2]};

  double* pt = &V[0];
  VectorXd VOut = Map<VectorXd>(pt, 6);

  return VOut;
}

Matrix<double, 4, 1> IRoboticArmBase::slerpQuaternion(Matrix<double, 4, 1>& q1, Matrix<double, 4, 1>& q2, double t) {
  Matrix<double, 4, 1> q;

  // Change sign of q2 if dot product of the two quaternions is negative => allows interpolating along the shortest path
  if (q1.dot(q2) < 0.0) {
    q2 = -q2;
  }

  double dotProduct = q1.dot(q2);
  if (dotProduct > 1.0) {
    dotProduct = 1.0;
  } else if (dotProduct < -1.0) {
    dotProduct = -1.0;
  }

  double omega = acos(dotProduct);

  if (fabs(omega) < numeric_limits<double>::epsilon()) {
    q = q1.transpose() + t * (q2 - q1).transpose();
  } else {
    q = (sin((1 - t) * omega) * q1 + sin(t * omega) * q2) / sin(omega);
  }

  return q;
}

Matrix<double, 4, 1> IRoboticArmBase::quaternionProduct(Matrix<double, 4, 1> q1, Matrix<double, 4, 1> q2) {
  Matrix<double, 4, 1> q;
  q(0) = q1(0) * q2(0) - (q1.segment(1, 3)).dot(q2.segment(1, 3));

  Matrix<double, 3, 1> q1Im = (q1.segment(1, 3));
  Matrix<double, 3, 1> q2Im = (q2.segment(1, 3));
  q.segment(1, 3) = q1(0) * q2Im + q2(0) * q1Im + q1Im.cross(q2Im);

  return q;
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
