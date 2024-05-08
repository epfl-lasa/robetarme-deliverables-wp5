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
    vector<double> actualJointSpeed = get<1>(stateJoints);

    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(firstQuatPos);
    // pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getDsQuatSpeed();

    Eigen::Vector3d centerLimitCycle;
    centerLimitCycle << firstQuatPos[4], firstQuatPos[5], firstQuatPos[6];
    Eigen::Vector3d limitCycleLinSpeed =
        dynamicalSystem_->updateLimitCycle3DPosVelWith2DLC(pairActualQuatPos.second, centerLimitCycle);
    pairQuatLinerSpeed.second = limitCycleLinSpeed;

    VectorXd twistDesiredEigen = dynamicalSystem_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    Eigen::VectorXd deltaTwist;
    deltaTwist = contactUpdateDS();

    vector<double> desiredJoint = roboticArm_->lowLevelControllerSF(stateJoints, twistDesiredEigen, deltaTwist);
    rosInterface_->sendState(desiredJoint);

    //publish to ros to ploting purpose
    VectorXd actualTwistEigen = roboticArm_->getTwistFromJointState(actualJoint, actualJointSpeed);
    vector<double> actualTwist(6, 0.0);
    for (int i = 0; i < actualTwistEigen.size(); ++i) {
      actualTwist[i] = actualTwistEigen[i];
    }
    vector<double> desiredTwist(6, 0.0);
    for (int i = 0; i < twistDesiredEigen.size(); ++i) {
      desiredTwist[i] = twistDesiredEigen[i];
    }
    rosInterface_->setCartesianTwist(actualTwist);
    rosInterface_->setDesiredDsTwist(desiredTwist);

    // rosLoop
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
    getRosLoopRate_()->sleep();
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
  double margin = 2;

  for (size_t i = 0; i < receivedWrench.size(); ++i) {
    receivedWrench[i] -= biasWrench_[i];
    if (receivedWrench[i] > margin) {
      outTwist(i) = receivedWrench[i] * 0.01;

    } else if (receivedWrench[i] < -margin) {
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
Eigen::VectorXd TaskSurfaceFinishing::contactUpdateDS() {
  vector<double> receivedWrench = rosInterface_->receiveWrench();
  Eigen::VectorXd outTwist(6);
  double alpha = 0.25;
  double margin = 1;
  double desiredContactForce = 10;

  receivedWrench[2] += desiredContactForce;

  for (size_t i = 0; i < receivedWrench.size(); ++i) {
    receivedWrench[i] -= biasWrench_[i];
  }

  if (receivedWrench[2] > margin) {
    outTwist(2) = receivedWrench[2] * 0.05;

  } else if (receivedWrench[2] < -margin) {
    outTwist(2) = receivedWrench[2] * 0.05;

  } else {
    outTwist(2) = 0;
  }

  for (size_t i = 0; i < outTwist.size(); ++i) {
    if (i != 2) {
      outTwist(i) = 0;
    }
  }

  outputTwist_ = alpha * outputTwist_ + (1 - alpha) * outTwist;

  if (outputTwist_(2) < -0.15) {
    outputTwist_(2) = -0.15;
  }
  if (outputTwist_(2) > 0.15) {
    outputTwist_(2) = 0.15;
  }

  return outputTwist_;
}
