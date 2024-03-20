#include "TaskSurfaceFinishing.h"

using namespace std;
using namespace Eigen;

TaskSurfaceFinishing::TaskSurfaceFinishing(ros::NodeHandle& nh, double freq, string robotName) :
    ITaskBase(nh, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();

  // Create an unique pointer for the instance of TargetExtraction
  targetExtraction_ = make_unique<TargetExtraction>(nodeHandle);

  // Create an unique pointer for the instance of PathPlanner
  pathPlanner_ = make_unique<PathPlanner>(nodeHandle);

  // Create an unique pointer for the instance of PathPlanner
  boustrophedonServer_ = make_unique<BoustrophedonServer>(nodeHandle);
}

bool TaskSurfaceFinishing::computePath() {
  cout << "computing path ..." << endl;

  // extract polygons for boustrophedon
  // WARNING: need the position of the target from Optitrack to continue
  vector<Vector3d> polygonsPositions = targetExtraction_->getPolygons();
  Quaterniond quatTarget = targetExtraction_->getQuatTarget();
  Vector3d posTarget = targetExtraction_->getPosTarget();
  targetExtraction_->seeTarget();

  // initialization
  pathPlanner_->setTarget(quatTarget, posTarget, polygonsPositions);
  double optimumRadius = pathPlanner_->getOptimumRadius();

  boustrophedonServer_->setOptimumRad(optimumRadius);
  // wait for the action server to startnew_rad

  cout << "Waiting for action server to start." << endl;
  boustrophedonServer_->initRosLaunch();

  cout << "Action server started" << endl;

  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal = pathPlanner_->ComputeGoal();
  boustrophedonServer_->polygonPub.publish(goal.property);

  cout << "Waiting for goal" << endl;

  nav_msgs::Path path;
  nav_msgs::Path pathTransformed;

  while (ros::ok() && !checkPath) {
    ros::Time start_time = ros::Time::now();
    pathPlanner_->publishInitialPose();
    goal.robot_position = pathPlanner_->getInitialPose();
    boustrophedonServer_->startPub.publish(goal.robot_position);
    boustrophedonServer_->client.sendGoal(goal);
    ROS_INFO_STREAM("Sending goal");

    // wait for the action to return
    bool finishedBeforeTimeout = boustrophedonServer_->client.waitForResult(ros::Duration(30.0));
    actionlib::SimpleClientGoalState state = boustrophedonServer_->client.getState();
    boustrophedon_msgs::PlanMowingPathResultConstPtr result = boustrophedonServer_->client.getResult();
    if (result->plan.points.size() < 1) {
      ROS_INFO("Action did not finish before the time out.");
    } else {
      ROS_INFO("Action finished: %s", state.toString().c_str());

      cout << "Result with : " << result->plan.points.size() << endl;

      if (result->plan.points.size() > 2) {
        pathPlanner_->convertStripingPlanToPath(result->plan, path);

        pathTransformed = pathPlanner_->getTransformedPath(path);

        vector<vector<double>> vectorPathTransformed = pathPlanner_->convertPathPlanToVectorVector(pathTransformed);

        vector<double> firstQuatPos = vectorPathTransformed[0];

        dynamicalSystem_->setPath(vectorPathTransformed);

        boustrophedonServer_->pathPub.publish(pathTransformed);

        boustrophedonServer_->closeRosLaunch();
        checkPath = true;
      }
    }
    ros::spinOnce();
    getRosLoopRate_().sleep();
  }
  cout << "path well compute" << endl;

  return checkPath;
}

bool TaskSurfaceFinishing::execute() {
  cout << "preforming shotcrete ..." << endl;
  dynamicalSystem_->resetInit();

  while (ros::ok() && !dynamicalSystem_->isFinished()) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getDsQuatSpeed();

    VectorXd twistDesiredEigen = dynamicalSystem_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);

    rosInterface_->sendState(desiredJointSpeed);

    ros::spinOnce();
    getRosLoopRate_().sleep();
  }

  return dynamicalSystem_->isFinished();
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
    getRosLoopRate_().sleep();
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
    getRosLoopRate_().sleep();

    //TODO: delet rviz dependency
    // twistMarker(twistDesiredEigen, pairActualQuatPos.second, pubDesiredVelFiltered_);
  }
  return dynamicalSystem_->checkLinearDs();
}
