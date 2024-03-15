#include "DynamicalSystem.h"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

//TODO: add some warning if not wellinit

DynamicalSystem::DynamicalSystem(double freq) {
  fs_ = freq;
  parameterInitialization();
}

void DynamicalSystem::parameterInitialization() {
  // Load parameters from YAML file
  string yaml_path = string(WP5_DYNAMICAL_SYSTEM_DIR) + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);

  // Access parameters from the YAML file
  CycleRadiusLC_ = config["limitCycleRadius"].as<double>();
  CycleSpeedLC_ = config["limitCycleSpeed"].as<double>();
  linearVelExpected_ = config["linearSpeed"].as<double>();
  ConvergenceRateLC_ = config["convRate"].as<double>();
  toleranceToNextPoint_ = config["toleranceToNextPoint"].as<double>();

  toolOffsetFromTarget_ = config["toolOffsetFromTarget"].as<double>();
  velocityLimit_ = config["velocityLimit"].as<double>();
}
std::vector<double> DynamicalSystem::getFirstQuatPos() { return firstQuatPos_; }

void DynamicalSystem::set_path(vector<vector<double>> pathInput) {
  desiredPath_ = pathInput;

  firstQuatPos_ = desiredPath_.front();
  lastQuatPos_ = desiredPath_.back();

  centerLimitCycle_(0) = firstQuatPos_[4];
  centerLimitCycle_(1) = firstQuatPos_[5];
  centerLimitCycle_(2) = firstQuatPos_[6];

  //--- here waiting for orientation control
  desiredOriVelocityFiltered_(0) = firstQuatPos_[0];
  desiredOriVelocityFiltered_(1) = firstQuatPos_[1];
  desiredOriVelocityFiltered_(2) = firstQuatPos_[2];
  desiredOriVelocityFiltered_(3) = firstQuatPos_[3];
}

void DynamicalSystem::setLimitCycleSpeedConv(double angSpeed, double conv) {
  ConvergenceRateLC_ = conv;
  CycleSpeedLC_ = angSpeed;
}
void DynamicalSystem::setLimitCycleRadius(double rad) { CycleRadiusLC_ = rad; }

void DynamicalSystem::setCartPose(pair<Quaterniond, Vector3d> pairQuatPos) {

  realQuat_ = pairQuatPos.first;
  realPos_ = pairQuatPos.second;

  //---- Update end effector pose (position+orientation)
  realQuatOffset_ = realQuat_;

  Quaterniond normalizedQuat = realQuat_.normalized();
  Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

  realPosOffset_ = realPos_ + toolOffsetFromTarget_ * rotation_matrix.col(2);
  if (iFollow_ == 0 && !init) {
    centerLimitCycle_ = realPosOffset_;
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

  dx = pathPoint(0) - realPosOffset_(0);
  dy = pathPoint(1) - realPosOffset_(1);
  dz = pathPoint(2) - realPosOffset_(2);

  norm = sqrt(dx * dx + dy * dy + dz * dz);
  scaleVel = linearVelExpected_ / norm;

  dVel(0) = dx * scaleVel;
  dVel(1) = dy * scaleVel;
  dVel(2) = dz * scaleVel;

  if (sqrt((pathPoint - realPosOffset_).norm()) <= toleranceToNextPoint_) {
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

  if (iFollow_ < desiredPath_.size()) {
    vector<double> desiredQuatPos = desiredPath_[iFollow_];
    pathPoint(0) = desiredQuatPos[4];
    pathPoint(1) = desiredQuatPos[5];
    pathPoint(2) = desiredQuatPos[6];

    dx = pathPoint(0) - realPosOffset_(0);
    dy = pathPoint(1) - realPosOffset_(1);
    dz = pathPoint(2) - realPosOffset_(2);

    norm = sqrt(dx * dx + dy * dy + dz * dz);
    scaleVel = linearVelExpected_ / norm;

    dVel(0) = dx * scaleVel;
    dVel(1) = dy * scaleVel;
    dVel(2) = dz * scaleVel;

    double dt = 1 / fs_;

    centerLimitCycle_ += dVel * dt;
    cerr << "target number: " << iFollow_ << endl;
    cerr << "error" << (sqrt((pathPoint - centerLimitCycle_).norm())) << endl;
    if (sqrt((pathPoint - centerLimitCycle_).norm()) <= toleranceToNextPoint_) {
      iFollow_ += 1;
    }
    updateLimitCycle3DPosVel_with2DLC(realPosOffset_, centerLimitCycle_);

  } else {

    dVel(0) = 0;
    dVel(1) = 0;
    dVel(2) = 0;
    desiredVel_(0) = 0;
    desiredVel_(1) = 0;
    desiredVel_(2) = 0;
    finish = true;
  }

  if (desiredVel_.norm() > velocityLimit_) {
    desiredVel_ = desiredVel_ / desiredVel_.norm() * velocityLimit_;
    cout << "TOO FAST!, limite speed =" << velocityLimit_ << endl;
  }
  // Fill desiredQuat with the values from desiredOriVelocityFiltered_
  Eigen::Quaterniond desiredQuat(desiredOriVelocityFiltered_(3), // w
                                 desiredOriVelocityFiltered_(0), // x
                                 desiredOriVelocityFiltered_(1), // y
                                 desiredOriVelocityFiltered_(2)  // z
  );

  return make_pair(desiredQuat, desiredVel_);
  // return make_pair(desiredQuat, dVel);
}

void DynamicalSystem::updateLimitCycle3DPosVel_with2DLC(Vector3d pos, Vector3d targetPoseCircleDS) {
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

  pos = pos - targetPoseCircleDS;
  for (size_t i = 0; i < 3; i++) {
    posEig(i) = pos(i);
  }
  posEig = rotMat.transpose() * posEig;
  for (int i = 0; i < 2; i++) posEig(i) = a[i] * posEig(i);

  double x_vel, y_vel, z_vel, R, T, cricle_plane_error;

  x_vel = 0;
  y_vel = 0;
  z_vel = -ConvergenceRateLC_ * posEig(2);

  R = sqrt(posEig(0) * posEig(0) + posEig(1) * posEig(1));
  T = atan2(posEig(1), posEig(0));

  double Rdot = -ConvergenceRateLC_ * (R - CycleRadiusLC_);
  double Tdot = CycleSpeedLC_;

  x_vel = Rdot * cos(T) - R * Tdot * sin(T);
  y_vel = Rdot * sin(T) + R * Tdot * cos(T);
  cricle_plane_error = posEig(2);

  velocity(0) = x_vel;
  velocity(1) = y_vel;
  velocity(2) = z_vel;

  velocity = rotMat * velocity;

  for (int i = 0; i < 3; i++) {
    desiredVel_[i] = velocity(i);
  }
}

void DynamicalSystem::setLinearSpeed(double speed) { linearVelExpected_ = speed; }
void DynamicalSystem::setToleranceNextPoint(double tol) { toleranceToNextPoint_ = tol; }
void DynamicalSystem::restartPath() { iFollow_ = 0; }
