#include "TaskSurfaceFinishing.h"

using namespace std;
using namespace Eigen;

TaskSurfaceFinishing::TaskSurfaceFinishing(ros::NodeHandle& nh, double freq, string robotName) :
    ITaskBase(nh, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();

  // Create an unique pointer for the instance of TargetExtraction
  targetExtraction_ = make_unique<TargetExtraction>(nodeHandle);

  // // Create an unique pointer for the instance of PathPlanner
  // pathPlanner_ = make_unique<PathPlanner>(nodeHandle);

  // // Create an unique pointer for the instance of PathPlanner
  // boustrophedonServer_ = make_unique<BoustrophedonServer>(nodeHandle);
  outputTwist_ = Eigen::VectorXd::Zero(6);
}

bool TaskSurfaceFinishing::computePath() {
  cout << "computing path ..." << endl;

  // extract polygons for boustrophedon
  // WARNING: need the position of the target from Optitrack to continue
  vector<Vector3d> polygonsPositions = targetExtraction_->getPolygons();
  Quaterniond quatTarget = targetExtraction_->getQuatTarget();
  Vector3d posTarget = targetExtraction_->getPosTarget();
  targetExtraction_->seeTarget();

  vector<double> targetQuatPos = {
      quatTarget.x(), quatTarget.y(), quatTarget.z(), quatTarget.w(), posTarget(0), posTarget(1), posTarget(2)};

  std::vector<std::vector<double>> result;
  for (int i = 0; i < 10; ++i) {
    result.push_back(targetQuatPos);
  }

  dynamicalSystem_->setPath(result);

  checkPath = true;

  return checkPath;
}

bool TaskSurfaceFinishing::execute() {
  set_bias();
  cout << "preforming limitcycle ..." << endl;

  dynamicalSystem_->resetInit();
  dynamicalSystem_->resetCheckLinearDs();

  vector<double> firstQuatPos = dynamicalSystem_->getFirstQuatPos();

  while (ros::ok()) {

    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(firstQuatPos);

    VectorXd twistDesiredEigen = dynamicalSystem_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    Eigen::Vector3d centerLimitCycle;
    centerLimitCycle << firstQuatPos[4], firstQuatPos[5], firstQuatPos[6];
    Eigen::Vector3d limitCycleLinSpeed =
        dynamicalSystem_->updateLimitCycle3DPosVelWith2DLC(pairActualQuatPos.second, centerLimitCycle);

    // twistDesiredEigen(0) = limitCycleLinSpeed(0);
    // twistDesiredEigen(1) = limitCycleLinSpeed(1);
    // twistDesiredEigen(2) = limitCycleLinSpeed(2);
    Eigen::VectorXd deltaTwist;
    deltaTwist = decoderWrench();

    vector<double> desiredJoint = roboticArm_->lowLevelControllerSF(stateJoints, twistDesiredEigen, deltaTwist);
    rosInterface_->sendState(desiredJoint);
    ros::spinOnce();
    getRosLoopRate_()->sleep();
  }
  return dynamicalSystem_->checkLinearDs();
}

void TaskSurfaceFinishing::set_bias() {
  int meanNum = 1000;
  std::cout << "Recording F/T sensor bias. Please do not touch the robot for 3 seconds..." << std::endl;

  // Initialize wrenchActual and receivedWrench vectors
  std::vector<double> wrenchActual(6, 0.0); // Assuming 6 elements in the wrench
  std::vector<double> receivedWrench;

  int meanIteration = 0;
  while (ros::ok() && (meanIteration < meanNum)) {
    receivedWrench = rosInterface_->receiveWrench();

    for (size_t i = 0; i < wrenchActual.size(); ++i) {
      wrenchActual[i] += receivedWrench[i] / meanNum;
    }
    meanIteration += 1;

    loopRate_.sleep();
  }

  // Assign the calculated bias to biasWrench_
  biasWrench_ = wrenchActual;

  // Print all elements of the biasWrench_ vector
  std::cout << "Recording F/T sensor bias done:" << std::endl;
  for (size_t i = 0; i < biasWrench_.size(); ++i) {
    std::cout << "biasWrench_[" << i << "] = " << biasWrench_[i] << std::endl;
  }
}

Eigen::VectorXd TaskSurfaceFinishing::decoderWrench() {
  vector<double> receivedWrench = rosInterface_->receiveWrench();
  Eigen::VectorXd outTwist(6);
  double alpha = 0.25;

  for (size_t i = 0; i < receivedWrench.size(); ++i) {
    receivedWrench[i] -= biasWrench_[i];
    if (receivedWrench[i] > 5) {
      outTwist(i) = receivedWrench[i] * 0.01;

    } else if (receivedWrench[i] < -5) {
      outTwist(i) = receivedWrench[i] * 0.01;

    } else {
      outTwist(i) = 0;
    }
  }

  outputTwist_ = alpha * outputTwist_ + (1 - alpha) * outTwist;

  for (size_t i = 0; i < outputTwist_.size(); ++i) {
    if (outputTwist_(i) < -0.15) {
      outputTwist_(i) = -0.15;
    }
    if (outputTwist_(i) > 0.15) {
      outputTwist_(i) = 0.15;
    }
  }
  return outputTwist_;
}

bool TaskSurfaceFinishing::TestSF() {
  dynamicalSystem_->resetInit();
  set_bias();
  vector<double> firstQuatPos = dynamicalSystem_->getFirstQuatPos();
  cout << "Go to first position, pointing on the target point:" << firstQuatPos[4] << firstQuatPos[5] << firstQuatPos[6]
       << endl;

  while (ros::ok() && !dynamicalSystem_->checkLinearDs()) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(firstQuatPos);

    VectorXd twistDesiredEigen = dynamicalSystem_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    //TEST
    Eigen::VectorXd deltaTwist;
    deltaTwist = decoderWrench();
    vector<double> desiredJoint = roboticArm_->lowLevelControllerSF(stateJoints, twistDesiredEigen, deltaTwist);
    //--------

    rosInterface_->sendState(desiredJoint);

    ros::spinOnce();
    getRosLoopRate_()->sleep();

    //TODO: delet rviz dependency
    // twistMarker(twistDesiredEigen, pairActualQuatPos.second, pubDesiredVelFiltered_);
  }
  return dynamicalSystem_->checkLinearDs();
}
