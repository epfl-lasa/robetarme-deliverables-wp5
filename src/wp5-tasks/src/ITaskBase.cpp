// clang off
#include "ITaskBase.h"
// clang on

#include "RoboticArmFactory.h"

using namespace std;
using namespace Eigen;

ITaskBase::ITaskBase(ros::NodeHandle& nh, double freq, string robotName) : nh_(nh), rosFreq_(freq), loopRate_(freq) {
  // Create an unique pointer for the instance of DynamicalSystem
  dynamicalSystem_ = make_unique<DynamicalSystem>(rosFreq_);

  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface_ = make_unique<RosInterfaceNoetic>(nh_, robotName);

  // Create an unique pointer for the instance of RosInterfaceNoetic
  RoboticArmFactory armFactory = RoboticArmFactory();
  roboticArm_ = armFactory.createRoboticArm(robotName);
}

bool ITaskBase::initialize() {
  if (roboticArm_) {
    cout << "----------------------" << roboticArm_->getName()
         << " chosen and well initializate----------------------------------" << endl;
    checkInitialization = true;
    homeJoint_ = roboticArm_->originalHomeJoint;
  } else {
    cout << "Error: roboticArm_ is null." << endl;
  }

  return checkInitialization;
}

double ITaskBase::getRosFrequency_() const { return rosFreq_; }

ros::Rate* ITaskBase::getRosLoopRate_() { return &loopRate_; }

ros::NodeHandle ITaskBase::getRosNodehandle_() const { return nh_; }

vector<double> ITaskBase::getHomeJoint_() const { return homeJoint_; }

void ITaskBase::setHomeJoint_(vector<double> desiredJoint) { homeJoint_ = desiredJoint; }

bool ITaskBase::goHomingPosition() {
  dynamicalSystem_->resetInit();
  dynamicalSystem_->resetCheckLinearDs();

  // get home position
  pair<Quaterniond, Vector3d> pairHomeQuatPos = roboticArm_->getFK(getHomeJoint_());
  Quaterniond homeQuat = pairHomeQuatPos.first;
  Vector3d homePos = pairHomeQuatPos.second;
  cout << homePos << endl;
  vector<double> desiredQuatPos = {
      homeQuat.x(), homeQuat.y(), homeQuat.z(), homeQuat.w(), homePos(0), homePos(1), homePos(2)};

  while (ros::ok() && !dynamicalSystem_->checkLinearDs()) {
    goToPoint(desiredQuatPos);

    ros::spinOnce();
    getRosLoopRate_()->sleep();
  }

  return dynamicalSystem_->checkLinearDs();
}

bool ITaskBase::goWorkingPosition() {
  dynamicalSystem_->resetInit();
  dynamicalSystem_->resetCheckLinearDs();

  vector<double> firstQuatPos = dynamicalSystem_->getFirstQuatPos();
  cout << "Go to first position, pointing on the target point:" << firstQuatPos[4] << firstQuatPos[5] << firstQuatPos[6]
       << endl;

  // // add the offsetof the distance between the endeffector and the target
  // vector<double> firstQuatPosOffset = dynamicalSystem_->addOffset(firstQuatPos);

  while (ros::ok() && !dynamicalSystem_->checkLinearDs()) {

    goToPoint(firstQuatPos);

    ros::spinOnce();
    getRosLoopRate_()->sleep();
  }

  return dynamicalSystem_->checkLinearDs();
}

bool ITaskBase::goToPoint(vector<double> firstQuatPosOffset) const {
  // set and get desired speed
  tuple<vector<double>, vector<double>, vector<double>> stateJoints;
  stateJoints = rosInterface_->receiveState();
  vector<double> actualJoint = get<0>(stateJoints);
  pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

  dynamicalSystem_->setCartPose(pairActualQuatPos);
  pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(firstQuatPosOffset);

  VectorXd twistDesiredEigen = dynamicalSystem_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

  vector<double> desiredJoint = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);
  rosInterface_->sendState(desiredJoint);

  return dynamicalSystem_->checkLinearDs();
}
