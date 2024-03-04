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
#include "IRoboticArmBase.h"


using namespace std;
using namespace Eigen;

/**
 * @brief Mother class to create all the prototype fonctions needed in the different robotic arms.
 *
 * This class provides methods to manage a robotic arm with all the necessary functions to control it.
 */


vector<double> IRoboticArmBase::getFK(vector<double> vectJoint) {
  Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());
  state_representation::JointPositions nextJoinState =  state_representation::JointPositions(robotName,jointNames,posJoint_eigen);        
  state_representation::CartesianPose nextCartesianPose = model->forward_kinematics(nextJoinState,tipLink);

  Vector3d p1Prime = nextCartesianPose.get_position();
  Quaterniond q1Prime = nextCartesianPose.get_orientation();

  vector<double> posCart ={q1Prime.x(),q1Prime.y(),q1Prime.z(),q1Prime.w(),p1Prime[0],p1Prime[1],p1Prime[2]};
  return posCart;
}

VectorXd IRoboticArmBase::getTwist(vector<double> posJoint, vector<double> speedJoint) {

  MatrixXd jacMatrix = getJacobian(posJoint);

  VectorXd speedJointEigen(nJoint);
  for(int i = 0 ;i<nJoint;++i){
    speedJointEigen(i) =speedJoint[i];
  }

  VectorXd twistLinearAngular = jacMatrix * speedJointEigen;
  return twistLinearAngular;
}

MatrixXd IRoboticArmBase::getJacobian(vector<double> vectJoint) {
  Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());
  state_representation::JointPositions actualJoinState = state_representation::JointPositions(robotName,jointNames,posJoint_eigen);  
  state_representation::Jacobian jacobianObject = model->compute_jacobian(actualJoinState); 
  MatrixXd jacobian = jacobianObject.data();

  return jacobian;  
}

vector<double> IRoboticArmBase::getIDynamics(vector<double> vectJoint, VectorXd speedEigen){
    vector<double> speedJointNext(nJoint);
    Matrix<double,6,1> twist = speedEigen;

    Vector3d  angular_velocity(3);
    angular_velocity << speedEigen(0),speedEigen(1),speedEigen(2);
    Vector3d  linear_velocity(3);
    linear_velocity << speedEigen(3),speedEigen(4),speedEigen(5);


    Map<VectorXd> posJoint_eigen(vectJoint.data(), vectJoint.size());

    state_representation::JointPositions actualJoinState = state_representation::JointPositions(robotName,jointNames,posJoint_eigen);       
    state_representation::CartesianTwist nextPostwist = state_representation::CartesianTwist(robotName, linear_velocity, angular_velocity, referenceFrame); 	
    state_representation::JointVelocities nextJoinStateSpeed = model->inverse_velocity(nextPostwist,actualJoinState,tipJoint);
    VectorXd speedJointNext_eigen = nextJoinStateSpeed.data() ;
    for (int i = 0; i < nJoint; ++i) {
        speedJointNext[i] = speedJointNext_eigen(i);

    }

    return speedJointNext;
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
      cout<<"no inverse kinematic found"<<endl;    
  }

  VectorXd posJointNextEigen = NextJointTask.data;
  vector<double> posJointNext ;
  for(int i = 0 ;i<nJoint;++i){    
      posJointNext[i] =posJointNextEigen(i);
      }
  //msgP.data = posJointNext;
  
  //actualJointTask.data.clear();
  pair<int, vector<double>> myPair = make_pair(rc, posJointNext);

  return myPair;
    
} 

void IRoboticArmBase::updateIK(double err ,double timeoutInSecs, string solveTypeStr ){
  // Convert the solve type string to the corresponding enum value
  TRAC_IK::SolveType solveType;
  if (solveTypeStr == "Distance") {
      solveType = TRAC_IK::Distance;
  } else if (solveTypeStr == "Speed") {
      solveType = TRAC_IK::Speed;
  } else {
      cout<< "Handle unrecognized solve types: set Distance as default value" << endl;
      solveType = TRAC_IK::Distance;
  }
  ikSolver= new TRAC_IK::TRAC_IK(baseLink, tipLink, paramURDF, timeoutInSecs, err, solveType);  
}

void IRoboticArmBase::initIK(){
  YAML::Node config = YAML::LoadFile("/../config/IK.yaml");
  // Get the solve type from the YAML file
  string solveTypeStr = config["IK/solve_type"].as<string>();

  // Convert the solve type string to the corresponding enum value
  TRAC_IK::SolveType solveType;
  if (solveTypeStr == "Distance") {
      solveType = TRAC_IK::Distance;
  } else if (solveTypeStr == "Speed") {
      solveType = TRAC_IK::Speed;
  } else {
      cout<< "Handle unrecognized solve types: set Distance as default value" << endl;
      solveType = TRAC_IK::Distance;
  }
  error = config["IK/error"].as<double>();
  timeoutInSecs = config["IK/timeoutInSecs"].as<double>();
}


VectorXd IRoboticArmBase::speed_func(vector<double> Pos, vector<double> quat2,vector<double> speed){
    
    //orientation
    Vector4d q1,q2 ;
    q1 << Pos[3], Pos[0],Pos[1],Pos[2]; //qw,qx,qy,qz
    q2 << quat2[3],quat2[0],quat2[1],quat2[2]; //qw,qx,qy,qz

    Vector4d dqd = Utils<double>::slerpQuaternion(q1, q2 ,0.5);    
    Vector4d deltaQ = dqd -  q1;

    Vector4d qconj = q1;
    qconj.segment(1,3) = -1 * qconj.segment(1,3);
    Vector4d temp_angVel = Utils<double>::quaternionProduct(deltaQ, qconj);

    Vector3d tmp_angular_vel = temp_angVel.segment(1,3);
    double maxDq = 0.2;
    if (tmp_angular_vel.norm() > maxDq)
        tmp_angular_vel = maxDq * tmp_angular_vel.normalized();

    double dsGain_ori = 0.50;
    double theta_gq = (-.5/(4*maxDq*maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
    Vector3d Omega_out  = 2 * dsGain_ori*(1+std::exp(theta_gq)) * tmp_angular_vel;
    
    vector<double> V = {Omega_out[0],Omega_out[1],Omega_out[2],speed[0],speed[1],speed[2]};

    double* pt = &V[0];
    VectorXd VOut = Map<VectorXd>(pt, 6);

    return VOut;
}