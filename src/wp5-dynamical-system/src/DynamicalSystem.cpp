#include "DynamicalSystem.h"

#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

//TODO: add some warning if not wellinit

DynamicalSystem::DynamicalSystem(double freq) {
  fs = freq;
  parameterInitialization();
}

void DynamicalSystem::parameterInitialization() {
  velocityLimit = 1.5;
  // Load parameters from YAML file
  string yaml_path = string(WP5_DYNAMICAL_SYSTEM_DIR) + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);

  // Access parameters from the YAML file
  CycleRadiusLC = config["limitCycleRadius"].as<double>();
  CycleSpeedLC = config["limit_cycle_speed"].as<double>();
  linearVelExpected = config["linear_speed"].as<double>();
  ConvergenceRateLC = config["conv_rate"].as<double>();
  toleranceToNextPoint = config["toleranceToNextPoint"].as<double>();

  toolOffsetFromTarget = config["toolOffsetFromTarget"].as<double>();
}
std::vector<double> DynamicalSystem::getFirstQuatPos() { return firstQuatPos; }

void DynamicalSystem::set_path(vector<vector<double>> pathInput) {
  desiredPath = pathInput;

  firstQuatPos = desiredPath.front();
  lastQuatPos = desiredPath.back();

  centerLimitCycle(0) = firstQuatPos[4];
  centerLimitCycle(1) = firstQuatPos[5];
  centerLimitCycle(2) = firstQuatPos[6];

  //--- here waiting for orientation control
  desiredOriVelocityFiltered_(0) = firstQuatPos[0];
  desiredOriVelocityFiltered_(1) = firstQuatPos[1];
  desiredOriVelocityFiltered_(2) = firstQuatPos[2];
  desiredOriVelocityFiltered_(3) = firstQuatPos[3];
}

void DynamicalSystem::setLimitCycleSpeedConv(double angSpeed, double conv) {
  ConvergenceRateLC = conv;
  CycleSpeedLC = angSpeed;
}
void DynamicalSystem::setLimitCycleRadius(double rad) { CycleRadiusLC = rad; }

void DynamicalSystem::setCartPose(pair<Quaterniond, Vector3d> pairQuatPos) {

  realQuat = pairQuatPos.first;
  realPos = pairQuatPos.second;

  //---- Update end effector pose (position+orientation)
  realQuatOffset = realQuat;

  Quaterniond normalizedQuat = realQuat.normalized();
  Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

  realPosOffset = realPos + toolOffsetFromTarget * rotation_matrix.col(2);
  if (iFollow == 0 && !init) {
    centerLimitCycle = realPosOffset;
    init = true;
  }
}
//----------------define all function-------------------------------------

// this function take the path comoute from server and create a linear DS
//when the eef is close the the next point it change the goal until the last point of the path
pair<Quaterniond, Vector3d> DynamicalSystem::getLinearDsOnePosition(vector<double> desiredQuatPos) {
  double dx, dy, dz;
  double norm;
  double scaleVel;
  Vector3d dVel;
  pathPoint(0) = desiredQuatPos[4];
  pathPoint(1) = desiredQuatPos[5];
  pathPoint(2) = desiredQuatPos[6];

  dx = pathPoint(0) - realPosOffset(0);
  dy = pathPoint(1) - realPosOffset(1);
  dz = pathPoint(2) - realPosOffset(2);

  norm = sqrt(dx * dx + dy * dy + dz * dz);
  scaleVel = linearVelExpected / norm;

  dVel(0) = dx * scaleVel;
  dVel(1) = dy * scaleVel;
  dVel(2) = dz * scaleVel;

  if (sqrt((pathPoint - realPosOffset).norm()) <= toleranceToNextPoint) {
    dVel(0) = 0;
    dVel(1) = 0;
    dVel(2) = 0;
    checkLinearDs = true;
  }

  // Fill desiredQuat with the values from desiredOriVelocityFiltered_
  Eigen::Quaterniond desiredQuat(desiredOriVelocityFiltered_(3), // w
                                 desiredOriVelocityFiltered_(0), // x
                                 desiredOriVelocityFiltered_(1), // y
                                 desiredOriVelocityFiltered_(2)  // z
  );

  return make_pair(desiredQuat, dVel);
}

pair<Quaterniond, Vector3d> DynamicalSystem::getDsQuatSpeed() {
  double dx, dy, dz;
  double norm;
  double scaleVel;
  Vector3d dVel;

  if (iFollow < desiredPath.size()) {
    vector<double> desiredQuatPos = desiredPath[iFollow];
    pathPoint(0) = desiredQuatPos[4];
    pathPoint(1) = desiredQuatPos[5];
    pathPoint(2) = desiredQuatPos[6];

    dx = pathPoint(0) - realPosOffset(0);
    dy = pathPoint(1) - realPosOffset(1);
    dz = pathPoint(2) - realPosOffset(2);

    norm = sqrt(dx * dx + dy * dy + dz * dz);
    scaleVel = linearVelExpected / norm;

    dVel(0) = dx * scaleVel;
    dVel(1) = dy * scaleVel;
    dVel(2) = dz * scaleVel;

    double dt = 1 / fs;

    centerLimitCycle += dVel * dt;
    cerr << "target number: " << iFollow << endl;
    cerr << "error" << (sqrt((pathPoint - centerLimitCycle).norm())) << endl;
    if (sqrt((pathPoint - centerLimitCycle).norm()) <= toleranceToNextPoint) {
      iFollow += 1;
    }
    updateLimitCycle3DPosVel_with2DLC(realPosOffset, centerLimitCycle);

  } else {

    dVel(0) = 0;
    dVel(1) = 0;
    dVel(2) = 0;
    desiredVel(0) = 0;
    desiredVel(1) = 0;
    desiredVel(2) = 0;
    finish = true;
  }

  if (desiredVel.norm() > velocityLimit) {
    desiredVel = desiredVel / desiredVel.norm() * velocityLimit;
    cout << "TOO FAST!, limite speed =" << velocityLimit << endl;
  }
  // Fill desiredQuat with the values from desiredOriVelocityFiltered_
  Eigen::Quaterniond desiredQuat(desiredOriVelocityFiltered_(3), // w
                                 desiredOriVelocityFiltered_(0), // x
                                 desiredOriVelocityFiltered_(1), // y
                                 desiredOriVelocityFiltered_(2)  // z
  );

  return make_pair(desiredQuat, desiredVel);
  // return make_pair(desiredQuat, dVel);
}

void DynamicalSystem::updateLimitCycle3DPosVel_with2DLC(Vector3d pos, Vector3d target_pose_cricleDS) {
  float a[2] = {1., 1.};
  float norm_a = sqrt(a[0] * a[0] + a[1] * a[1]);
  for (int i = 0; i < 2; i++) a[i] = a[i] / norm_a;

  Vector3d velocity;
  Vector3d posEig;

  //--- trans real ori to rotation matrix
  Quaterniond new_quat;
  new_quat.w() = desiredOriVelocityFiltered_(3);
  new_quat.x() = desiredOriVelocityFiltered_(0);
  new_quat.y() = desiredOriVelocityFiltered_(1);
  new_quat.z() = desiredOriVelocityFiltered_(2);
  Matrix3d rotMat = new_quat.toRotationMatrix();

  // cerr<<"pose: "<< pose(0) <<","<< pose(1) <<","<< pose(2) <<"," << endl;
  // cerr<<"target_pose_cricleDS: "<< target_pose_cricleDS(0) <<","<< target_pose_cricleDS(1) <<","<< target_pose_cricleDS(2) <<"," << endl;

  pos = pos - target_pose_cricleDS;
  for (size_t i = 0; i < 3; i++) {
    posEig(i) = pos(i);
  }
  posEig = rotMat.transpose() * posEig;
  for (int i = 0; i < 2; i++) posEig(i) = a[i] * posEig(i);

  double x_vel, y_vel, z_vel, R, T, cricle_plane_error;

  x_vel = 0;
  y_vel = 0;
  z_vel = -ConvergenceRateLC * posEig(2);

  R = sqrt(posEig(0) * posEig(0) + posEig(1) * posEig(1));
  T = atan2(posEig(1), posEig(0));

  double Rdot = -ConvergenceRateLC * (R - CycleRadiusLC);
  double Tdot = CycleSpeedLC;

  x_vel = Rdot * cos(T) - R * Tdot * sin(T);
  y_vel = Rdot * sin(T) + R * Tdot * cos(T);
  cricle_plane_error = posEig(2);

  velocity(0) = x_vel;
  velocity(1) = y_vel;
  velocity(2) = z_vel;

  velocity = rotMat * velocity;

  for (int i = 0; i < 3; i++) {
    desiredVel[i] = velocity(i);
  }
}

void DynamicalSystem::setLinearSpeed(double speed) { linearVelExpected = speed; }
void DynamicalSystem::setToleranceNextPoint(double tol) { toleranceToNextPoint = tol; }
void DynamicalSystem::restartPath() { iFollow = 0; }
