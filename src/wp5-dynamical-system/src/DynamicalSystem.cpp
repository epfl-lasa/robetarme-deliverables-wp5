#include "DynamicalSystem.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

using namespace std;
using namespace Eigen;

DynamicalSystem::DynamicalSystem(double freq) {
  fs_ = freq;
  parameterInitialization();
}

void DynamicalSystem::parameterInitialization() {

  string alternativeYamlPath = string(WP5_DYNAMICAL_SYSTEM_DIR) + "/config/control_config.yaml";
  string yamlPath = string(WP5_DYNAMICAL_SYSTEM_DIR) + "/../../config/control_config.yaml";

  // Check if the alternative YAML file exists
  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << yamlPath << endl;
  } else {
    yamlPath = alternativeYamlPath;
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);

  // Access parameters from the YAML file
  cycleRadiusLC_ = config["limitCycleRadius"].as<double>();
  cycleSpeedLC_ = config["limitCycleSpeed"].as<double>();
  linearVelExpected_ = config["linearSpeed"].as<double>();
  convergenceRateLC_ = config["convRate"].as<double>();
  toleranceToNextPoint_ = config["toleranceToNextPoint"].as<double>();

  toolOffsetFromTarget_ = config["toolOffsetFromTarget"].as<double>();
  velocityLimit_ = config["velocityLimit"].as<double>();
}
void DynamicalSystem::setOffset(double offset) { toolOffsetFromTarget_ = offset; }

void DynamicalSystem::setPath(vector<vector<double>> pathInput) {

  vector<vector<double>> pathOff(pathInput.size());
  // adding offset for the endefector distance with the target to the entire path
  for (size_t i = 0; i < pathInput.size(); ++i) {
    pathOff[i] = addOffset(pathInput[i]);
  }

  desiredPath_ = pathOff;
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

void DynamicalSystem::setCartPose(pair<Quaterniond, Vector3d> pairQuatPos) {
  realQuat_ = pairQuatPos.first;
  realPos_ = pairQuatPos.second;

  if (iFollow_ == 0 && !init_) {
    centerLimitCycle_ = realPos_;
    init_ = true;
  }
}

vector<double> DynamicalSystem::addOffset(vector<double> desiredQuatPos) {

  Vector3d PosNoOffset;
  PosNoOffset << desiredQuatPos[4], desiredQuatPos[5], desiredQuatPos[6];
  Eigen::Quaterniond QuatNoOffset(desiredQuatPos[3], desiredQuatPos[0], desiredQuatPos[1], desiredQuatPos[2]);
  Quaterniond normalizedQuat = QuatNoOffset.normalized();
  Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

  Vector3d PosOffset = PosNoOffset - toolOffsetFromTarget_ * rotation_matrix.col(2);
  vector<double> desiredQuatPosOffset = {desiredQuatPos[0],
                                         desiredQuatPos[1],
                                         desiredQuatPos[2],
                                         desiredQuatPos[3],
                                         PosOffset(0),
                                         PosOffset(1),
                                         PosOffset(2)};
  return desiredQuatPosOffset;
}

// void DynamicalSystem::setBiasForce(VectorXd meanWrenchFromSensor) { meanWrenchFromSensor_ = meanWrenchFromSensor; }
void DynamicalSystem::setLinearSpeed(double speed) { linearVelExpected_ = speed; }

void DynamicalSystem::setLimitCycleSpeedConv(double angSpeed, double conv) {
  convergenceRateLC_ = conv;
  cycleSpeedLC_ = angSpeed;
}

void DynamicalSystem::setLimitCycleRadius(double rad) { cycleRadiusLC_ = rad; }

void DynamicalSystem::setToleranceNextPoint(double tol) { toleranceToNextPoint_ = tol; }

void DynamicalSystem::setToolOffsetFromTarget_(double offset) { toolOffsetFromTarget_ = offset; }

bool DynamicalSystem::isFinished() const { return finish_; }

bool DynamicalSystem::isInitialized() const { return init_; }

bool DynamicalSystem::checkLinearDs() const { return checkLinearDs_; }

// this function take the path comoute from server and create a linear DS
//when the eef is close the the next point it change the goal until the last point of the path
pair<Quaterniond, Vector3d> DynamicalSystem::getLinearDsOnePosition(vector<double> desiredQuatPos) {
  double dx, dy, dz;
  double norm;
  double scaleVel;
  Vector3d dVel;
  checkLinearDs_ = false;

  // Fill desiredQuat with the values from desiredOriVelocityFiltered_
  Eigen::Quaterniond desiredQuat(desiredQuatPos[3], // w
                                 desiredQuatPos[0], // x
                                 desiredQuatPos[1], // y
                                 desiredQuatPos[2]  // z
  );
  desiredQuat_ = desiredQuat;

  pathPoint_(0) = desiredQuatPos[4];
  pathPoint_(1) = desiredQuatPos[5];
  pathPoint_(2) = desiredQuatPos[6];

  dx = pathPoint_(0) - realPos_(0);
  dy = pathPoint_(1) - realPos_(1);
  dz = pathPoint_(2) - realPos_(2);

  // Calculate the difference between the two quaternions
  Eigen::Quaterniond relativeRotation = desiredQuat_ * realQuat_.conjugate();
  // Calculate the norm of the difference
  double normQuat = relativeRotation.norm();

  // Output the result
  // std::cout << "The normQuat of the difference between q1 and q2 is: " << normQuat << std::endl;

  norm = sqrt(dx * dx + dy * dy + dz * dz);
  scaleVel = linearVelExpected_ / norm;

  dVel(0) = dx * scaleVel;
  dVel(1) = dy * scaleVel;
  dVel(2) = dz * scaleVel;

  if (sqrt((pathPoint_ - realPos_).norm()) <= toleranceToNextPoint_) {
    dVel(0) = 0;
    dVel(1) = 0;
    dVel(2) = 0;
    checkLinearDs_ = true;
  }

  return make_pair(desiredQuat_, dVel);
}

pair<Quaterniond, Vector3d> DynamicalSystem::getDsQuatSpeed() {
  double dx, dy, dz;
  double norm;
  double scaleVel;
  Vector3d dVel;
  dVel.setZero();
  Vector3d pathPointNext;
  pathPointNext.setZero();

  if (iFollow_ < desiredPath_.size() - 1) {

    desiredQuat_.w() = desiredPath_[iFollow_][3];
    desiredQuat_.x() = desiredPath_[iFollow_][0];
    desiredQuat_.y() = desiredPath_[iFollow_][1];
    desiredQuat_.z() = desiredPath_[iFollow_][2];
    pathPointNext(0) = desiredPath_[iFollow_][4];
    pathPointNext(1) = desiredPath_[iFollow_][5];
    pathPointNext(2) = desiredPath_[iFollow_][6];

    //use the point betweeen path as a linear DS
    dx = pathPointNext(0) - centerLimitCycle_(0);
    dy = pathPointNext(1) - centerLimitCycle_(1);
    dz = pathPointNext(2) - centerLimitCycle_(2);

    norm = sqrt(dx * dx + dy * dy + dz * dz);
    scaleVel = linearVelExpected_ / norm;

    dVel(0) = dx * scaleVel;
    dVel(1) = dy * scaleVel;
    dVel(2) = dz * scaleVel;

    double dt = 1 / fs_;
    double dxcheck = 0, dycheck, dzcheck;
    double normcheck = 0;

    //check if the eef is folowing the limit cy0cle
    // dxcheck = centerLimitCycle_(0) - realPos_[0];
    // dycheck = centerLimitCycle_(1) - realPos_[1];
    // dzcheck = centerLimitCycle_(2) - realPos_[2];
    // normcheck = sqrt(dxcheck * dxcheck + dycheck * dycheck + dzcheck * dzcheck);

    // if(normcheck < 1.2 * cycleRadiusLC_){
    //   centerLimitCycle_ += dVel * dt;
    //   cout <<"inside"<<endl;
    // }
    centerLimitCycle_ += dVel * dt;

    // cout <<"normcheck"<<normcheck<<endl;

    // cerr << "target number: " << iFollow_ << endl;
    // cerr << "error" << (sqrt((pathPoint_ - centerLimitCycle_).norm())) << endl;
    if (sqrt((pathPointNext - centerLimitCycle_).norm()) <= toleranceToNextPoint_) {
      iFollow_ += 1;
      cout << "-----------------------------------target number: " << iFollow_ << "reached" << endl;
    }
    updateLimitCycle3DPosVelWith2DLC(realPos_, centerLimitCycle_);

  } else {

    dVel(0) = 0;
    dVel(1) = 0;
    dVel(2) = 0;
    desiredVel_(0) = 0;
    desiredVel_(1) = 0;
    desiredVel_(2) = 0;
    finish_ = true;
  }

  if (desiredVel_.norm() > velocityLimit_) {
    desiredVel_ = desiredVel_ / desiredVel_.norm() * velocityLimit_;
    cout << "TOO FAST!, limite speed =" << velocityLimit_ << endl;
  }

  return make_pair(desiredQuat_, desiredVel_);
  // return make_pair(desiredQuat_, dVel);
}

std::vector<double> DynamicalSystem::getFirstQuatPos() const { return firstQuatPos_; }

VectorXd DynamicalSystem::getTwistFromDS(Quaterniond quat1, pair<Quaterniond, Vector3d> pairQuatPos) {
  Quaterniond quat2 = pairQuatPos.first;
  Vector3d speed = pairQuatPos.second;

  //orientation
  Vector4d q1, q2;
  q1 << quat1.w(), quat1.x(), quat1.y(), quat1.z(); //qw,qx,qy,qz
  q2 << quat2.w(), quat2.x(), quat2.y(), quat2.z(); //qw,qx,qy,qz

  Vector4d dqd = slerpQuaternion_(q1, q2, 0.5);
  Vector4d deltaQ = dqd - q1;

  Vector4d qconj = q1;
  qconj.segment(1, 3) = -1 * qconj.segment(1, 3);
  Vector4d tempAngVel = quaternionProduct_(deltaQ, qconj);

  Vector3d tmpAngularVel = tempAngVel.segment(1, 3);
  double maxDq = 0.2;
  if (tmpAngularVel.norm() > maxDq)
    tmpAngularVel = maxDq * tmpAngularVel.normalized();

  double dsGainOri = 0.5;
  double thetaGq = (-.5 / (4 * maxDq * maxDq)) * tmpAngularVel.transpose() * tmpAngularVel;
  Vector3d omegaOut = 2 * dsGainOri * (1 + exp(thetaGq)) * tmpAngularVel;

  vector<double> v = {speed(0), speed(1), speed(2), omegaOut[0], omegaOut[1], omegaOut[2]};

  double* pt = &v[0];
  VectorXd vOut = Map<VectorXd>(pt, 6);

  return vOut;
}

Vector3d DynamicalSystem::updateLimitCycle3DPosVelWith2DLC(Vector3d pos, Vector3d targetPoseCircleDS) {
  float a[2] = {1., 1.};
  float norm_a = sqrt(a[0] * a[0] + a[1] * a[1]);
  for (int i = 0; i < 2; i++) a[i] = a[i] / norm_a;

  Vector3d velocity;
  Vector3d posEig;

  //--- trans real ori to rotation matrix
  Quaterniond new_quat = desiredQuat_;
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
  z_vel = -convergenceRateLC_ * posEig(2);

  R = sqrt(posEig(0) * posEig(0) + posEig(1) * posEig(1));
  T = atan2(posEig(1), posEig(0));

  double Rdot = -convergenceRateLC_ * (R - cycleRadiusLC_);
  double tDot = cycleSpeedLC_;

  x_vel = Rdot * cos(T) - R * tDot * sin(T);
  y_vel = Rdot * sin(T) + R * tDot * cos(T);
  cricle_plane_error = posEig(2);

  velocity(0) = x_vel;
  velocity(1) = y_vel;
  velocity(2) = z_vel;

  velocity = rotMat * velocity;

  for (int i = 0; i < 3; i++) {
    desiredVel_[i] = velocity(i);
  }
  return velocity;
}
void DynamicalSystem::resetInit() { init_ = false; };

void DynamicalSystem::resetCheckLinearDs() { checkLinearDs_ = false; };

void DynamicalSystem::restartPath() { iFollow_ = 0; }

Matrix<double, 4, 1> DynamicalSystem::slerpQuaternion_(Matrix<double, 4, 1>& q1, Matrix<double, 4, 1>& q2, double t) {
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

Matrix<double, 4, 1> DynamicalSystem::quaternionProduct_(Matrix<double, 4, 1> q1, Matrix<double, 4, 1> q2) {
  Matrix<double, 4, 1> q;
  q(0) = q1(0) * q2(0) - (q1.segment(1, 3)).dot(q2.segment(1, 3));

  Matrix<double, 3, 1> q1Im = (q1.segment(1, 3));
  Matrix<double, 3, 1> q2Im = (q2.segment(1, 3));
  q.segment(1, 3) = q1(0) * q2Im + q2(0) * q1Im + q1Im.cross(q2Im);

  return q;
}
