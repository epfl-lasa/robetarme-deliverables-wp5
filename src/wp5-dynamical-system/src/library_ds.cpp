#include "library_ds.h"


DynamicalSystem::DynamicalSystem(double freq)
{
  fs = freq;
  parameter_initialization();
}

void DynamicalSystem::parameter_initialization(){
  velocityLimit=1.5;
  // Load parameters from YAML file
  std::string yaml_path = + "/../config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);
  // Access parameters from the YAML file
  robotName = config["robotName"].as<std::string>();
  CycleRadiusLC = config["limit_cycle_radius"].as<double>();
  CycleSpeedLC = config["limit_cycle_speed"].as<double>();
  linearVelExpected = config["linear_speed"].as<double>();
  ConvergenceRateLC =config["conv_rate"].as<double>();
  toleranceToNextPoint =config["toleranceToNextPoint"].as<double>();

  toolOffsetFromTarget = config["toolOffsetFromTarget"].as<double>();
}

void DynamicalSystem::set_path(std::vector<std::vector<double>> pathInput )
{ 
  desiredPath = pathInput;

  firstQuatPos = desiredPath.front();
  lastQuatPos = desiredPath.back();

  centerLimitCycle(0)=firstQuatPos[4];
  centerLimitCycle(1)=firstQuatPos[5];
  centerLimitCycle(2)=firstQuatPos[6];
  //--- here waiting for orinetation control
  desiredOriVelocityFiltered_(0) = firstQuatPos[0];
  desiredOriVelocityFiltered_(1) = firstQuatPos[1];
  desiredOriVelocityFiltered_(2) = firstQuatPos[2];
  desiredOriVelocityFiltered_(3) = firstQuatPos[3];
}

void DynamicalSystem::set_limitCycle_speed_conv(double angSpeed, double conv)
{
  ConvergenceRateLC = conv;
  CycleSpeedLC      = angSpeed;
}
void DynamicalSystem::set_limitCycle_radius(double rad)
{
  CycleRadiusLC    = rad;
}


void DynamicalSystem::addOffsetEef(Eigen::Vector3d pos, Eigen::Vector4d quat) {

  realPos(0) = pos(0);
  realPos(1) = pos(1);
  realPos(2) = pos(2);

  realQuat = Eigen::Quaterniond(quat(3),quat(0),quat(1),quat(2));

  //---- Update end effector pose (position+orientation)
  realQuatOffset = realQuat;

  Eigen::Quaterniond normalizedQuat = realQuat.normalized();
  Eigen::Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

  realPosOffset = realPos + toolOffsetFromTarget*rotation_matrix.col(2);
}
  //----------------define all function-------------------------------------
   
    // this function take the path comoute from server and create a linear DS
    //when the eef is close the the next point it change the goal until the last point of the path
Eigen::Vector3d DynamicalSystem::get_DS_vel()
{ 
  double dx,dy,dz;
  double norm;
  double scaleVel;
  Eigen::Vector3d dVel;
  Eigen::Vector3d pathPoint;
  
  if (iFollow < desiredPath.size() - 1)
  {
    std::vector<double> desiredQuatPos = desiredPath[iFollow + 1 ];
    pathPoint(0)=desiredQuatPos[4];
    pathPoint(1)=desiredQuatPos[5];
    pathPoint(2)=desiredQuatPos[6];
  
    dx = pathPoint(0) - realPosOffset(0);
    dy = pathPoint(1) - realPosOffset(1);
    dz = pathPoint(2) - realPosOffset(2);

    norm = sqrt(dx*dx+dy*dy+dz*dz);
    scaleVel = linearVelExpected/norm;

    dVel(0)=dx*scaleVel;
    dVel(1)=dy*scaleVel;
    dVel(2)=dz*scaleVel;

    double dt = 1/fs;
    
    centerLimitCycle+=dVel*dt;
    

    std::cerr<<"target number: "<<iFollow<< std::endl;
    std::cerr<<"error"<<(std::sqrt((pathPoint - centerLimitCycle).norm()))<< std::endl;
    if (std::sqrt((pathPoint - centerLimitCycle).norm())<=toleranceToNextPoint)
    {
      iFollow+=1;
    }
    updateLimitCycle3DPosVel_with2DLC(realPosOffset,centerLimitCycle);

  }else
  {
    pathPoint(0)=desiredPath.poses[iFollow].pose.position.x;
    pathPoint(1)=desiredPath.poses[iFollow].pose.position.y;
    pathPoint(2)=desiredPath.poses[iFollow].pose.position.z;

    dx = pathPoint(2) - realPosOffset(0);
    dy = pathPoint(1) - realPosOffset(1);
    dz = pathPoint(0) - realPosOffset(2);

    norm = sqrt(dx*dx+dy*dy+dz*dz);
    scaleVel = linearVelExpected/norm;

    dVel(0)=0;
    dVel(1)=0;
    dVel(2)=0;
    desiredVel(0)=0;
    desiredVel(1)=0;
    desiredVel(2)=0;
    finish =true;
  }

  if (desiredVel.norm() > velocityLimit) {
    desiredVel = desiredVel / desiredVel.norm() * velocityLimit;
    std::cout << "TOO FAST!, limite speed ="<< velocityLimit<< std::endl;
  }
  return desiredVel;
}

// void DynamicalSystem::publishPointStamped(const Eigen::Vector3d&  pathPoint ) {

// geometry_msgs::PointStamped point_stamped_msg;
// point_stamped_msg.header.stamp = ros::Time::now();
// point_stamped_msg.header.frame_id = "base"; // Set your desired frame_id

// // Assign Eigen vector components to PointStamped message
// point_stamped_msg.point.x = pathPoint(0);
// point_stamped_msg.point.y = pathPoint(1);
// point_stamped_msg.point.z = pathPoint(2);

// // Publish the PointStamped message
// point_pub.publish(point_stamped_msg);
// }


void DynamicalSystem::updateLimitCycle3DPosVel_with2DLC(Eigen::Vector3d pose, Eigen::Vector3d target_pose_cricleDS) 
{
  float a[2] = {1., 1.};
  float norm_a=std::sqrt(a[0]*a[0]+a[1]*a[1]);
  for (int i=0; i<2; i++)
      a[i]=a[i]/norm_a;

  Eigen::Vector3d velocity;
  Eigen::Vector3d pose_eig;

  //--- trans real ori to rotation matrix
  Eigen::Quaterniond new_quat;
  new_quat.w()=desiredOriVelocityFiltered_(3);
  new_quat.x()=desiredOriVelocityFiltered_(0);
  new_quat.y()=desiredOriVelocityFiltered_(1);
  new_quat.z()=desiredOriVelocityFiltered_(2);
  Eigen::Matrix3d rotMat = new_quat.toRotationMatrix();

  // std::cerr<<"pose: "<< pose(0) <<","<< pose(1) <<","<< pose(2) <<"," << std::endl;
  // std::cerr<<"target_pose_cricleDS: "<< target_pose_cricleDS(0) <<","<< target_pose_cricleDS(1) <<","<< target_pose_cricleDS(2) <<"," << std::endl;

  pose = pose-target_pose_cricleDS;
  for (size_t i = 0; i < 3; i++)
  {
    pose_eig(i)=pose(i);
  }
  pose_eig = rotMat.transpose() * pose_eig;
  for (int i=0; i<2; i++)
      pose_eig(i) = a[i] * pose_eig(i);

  double x_vel,y_vel,z_vel,R,T,cricle_plane_error;

  x_vel = 0;
  y_vel = 0;
  z_vel = - ConvergenceRateLC * pose_eig(2);

  R = sqrt(pose_eig(0) * pose_eig(0) + pose_eig(1) * pose_eig(1));
  T = atan2(pose_eig(1), pose_eig(0));

  double Rdot = - ConvergenceRateLC * (R - CycleRadiusLC);
  double Tdot = CycleSpeedLC;

  x_vel = Rdot * cos(T) - R * Tdot * sin(T);
  y_vel = Rdot * sin(T) + R * Tdot * cos(T);
  cricle_plane_error=pose_eig(2);

  velocity(0) = x_vel;
  velocity(1) = y_vel;
  velocity(2) = z_vel;

  velocity=rotMat*velocity;

  for(int i=0; i<3; i++){
    desiredVel[i] = velocity(i);
  }
}

void DynamicalSystem::set_linear_speed(double speed){ 
  linearVelExpected = speed;
}
void DynamicalSystem::set_tolerance_next_point(double tol){ 
  toleranceToNextPoint = tol;
}
void DynamicalSystem::restart_path(){
  iFollow = 0;
}
