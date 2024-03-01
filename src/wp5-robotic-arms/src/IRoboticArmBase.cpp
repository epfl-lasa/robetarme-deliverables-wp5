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
#include "IRoboticArmBase.h"


using namespace std;

/**
 * @brief Mother class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */

vector<double> IRoboticArmBase::getFK(vector<double> joint_positions) {
  Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());
  state_representation::JointPositions nextJoinState =  state_representation::JointPositions(robot_name,joint_names,posJoint_eigen);        
  state_representation::CartesianPose nextCartesianPose = model->forward_kinematics(nextJoinState,tipLink);

  Vector3d p1Prime = nextCartesianPose.get_position();
  Quaterniond q1Prime = nextCartesianPose.get_orientation();

  vector<double> posCart ={q1Prime.x(),q1Prime.y(),q1Prime.z(),q1Prime.w(),p1Prime[0],p1Prime[1],p1Prime[2]};
  return posCart;
}

geometry_msgs::Twist IRoboticArmBase::getTwist(vector<double> joint_positions, vector<double> joint_velocities) {
  // Create a Twist message
  geometry_msgs::Twist twist;

  MatrixXd jacMatrix = getJacobian(posJoint);

  VectorXd speedJointEigen(nJoint);
  for(int i = 0 ;i<nJoint;++i){
    speedJointEigen(i) =speedJoint[i];
  }

  VectorXd result = jacMatrix * speedJointEigen;

  // Populate linear velocities
  twist.linear.x = result(0);
  twist.linear.y = result(1);
  twist.linear.z = result(2);

  // Populate angular velocities
  twist.angular.x = result(3);
  twist.angular.y = result(4);
  twist.angular.z = result(5); // Angular velocity around the z-axis

  return twist;
}

MatrixXd IRoboticArmBase::getJacobian(vector<double> joint_positions) {
  Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());
  state_representation::JointPositions actualJoinState = state_representation::JointPositions(robot_name,joint_names,posJoint_eigen);  
  state_representation::Jacobian jacobianObject = model->compute_jacobian(actualJoinState); 
  MatrixXd jacobian = jacobianObject.data();

  return jacobian;  
}


pair<int, vector<double>> IRoboticArmBase::getIK(vector<double> actualJoint, vector<double> vectorQuatPos ) {  
    
    //Inverse kinematics trac-IK
    KDL::JntArray NextJointTask;
    KDL::JntArray actualJointTask; 

    VectorXd pos_joint_actual_eigen(nJoint);
    for(int i = 0 ;i<nJoint;++i){
        pos_joint_actual_eigen(i) =actualJoint[i];
    }
    actualJointTask.data = pos_joint_actual_eigen;

    KDL::Vector Vec(vectorQuatPos[4],vectorQuatPos[5],vectorQuatPos[6]);

    Quaterniond q(vectorQuatPos[3],vectorQuatPos[0],vectorQuatPos[1],vectorQuatPos[2]);
    q.normalize();
    KDL::Rotation Rot = KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w());
    KDL::Frame NextJointCartesian(Rot,Vec); 
    rc = ikSolver->CartToJnt(actualJointTask, NextJointCartesian, NextJointTask);
    if (rc< 0){
        ROS_INFO("no inverse kinematic found");    
    }

    VectorXd posJointNextEigen = NextJointTask.data;

    for(int i = 0 ;i<nJoint;++i){    
        posJointNext[i] =posJointNextEigen(i);
        }
    //msgP.data = posJointNext;
    
    //actualJointTask.data.clear();
    pair<int, vector<double>> myPair = make_pair(rc, posJointNext);

    return myPair;
    
} 

void IRoboticArmBase::updateIK(double err ,double timeoutInSecs ){
    ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, URDF_param, timeoutInSecs, err, type);  
}