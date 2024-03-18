// clang off
#include "ITaskBase.h"
// clang on

//TODO(Tristant): remove rviz dependency
#include <geometry_msgs/Point.h>        //<----------to remove
#include <geometry_msgs/PointStamped.h> //<----------to remove
#include <yaml-cpp/yaml.h>

#include "RoboticArmIiwa7.h"
#include "RoboticArmUr5.h"
#include "visualization_msgs/Marker.h"

using namespace std;
using namespace Eigen;

//TODO(Tristant): RVIZ DEP
void twistMarker(VectorXd twistDesiredEigen, Vector3d pos, ros::Publisher& markerPub) {
  std::array<double, 3> scaleLinear = {0.01, 0.1, 0.5};
  std::array<double, 4> colorLinear = {1.0, 1.0, 0.0, 1.0}; // rgba, don't forget to set alpha
  std::array<double, 3> colorAngular = {1.0, 0.0, 0.0};     // rgb, alpha is taken from linear

  visualization_msgs::Marker linearMarker, angularMarker;

  // Linear twist arrow marker
  linearMarker.header.frame_id = "iiwa_link_0"; // Set your desired frame ID
  linearMarker.header.stamp = ros::Time();
  linearMarker.ns = "twist";
  linearMarker.id = 0;
  linearMarker.type = visualization_msgs::Marker::ARROW;
  linearMarker.action = visualization_msgs::Marker::ADD;
  linearMarker.color.r = colorLinear[0];
  linearMarker.color.g = colorLinear[1];
  linearMarker.color.b = colorLinear[2];
  linearMarker.color.a = colorLinear[3];

  linearMarker.scale.x = scaleLinear[0]; // Arrow width
  linearMarker.scale.y = scaleLinear[1]; // Arrow head width
  linearMarker.scale.z = scaleLinear[2]; // Arrow head length

  linearMarker.pose.orientation.w = 1.0;
  linearMarker.pose.position.x = pos(0);
  linearMarker.pose.position.y = pos(1);
  linearMarker.pose.position.z = pos(2);

  linearMarker.points.push_back(geometry_msgs::Point());
  geometry_msgs::Point point;
  point.x = twistDesiredEigen(3);
  point.y = twistDesiredEigen(4);
  point.z = twistDesiredEigen(5);
  linearMarker.points.push_back(point);

  // Angular twist arrow marker
  angularMarker = linearMarker; // Copy settings from linearMarker
  angularMarker.id += 1;
  angularMarker.color.r = colorAngular[0];
  angularMarker.color.g = colorAngular[1];
  angularMarker.color.b = colorAngular[2];

  // Angular twist direction
  angularMarker.points.push_back(geometry_msgs::Point());
  point.x = twistDesiredEigen(0);
  point.y = twistDesiredEigen(1);
  point.z = twistDesiredEigen(2);
  angularMarker.points.push_back(point);

  // Publish markers
  markerPub.publish(linearMarker);
  //markerPub.publish(angularMarker);
}

void publishPointStamped(const Vector3d& pathPoint, ros::Publisher pointPub) {
  geometry_msgs::PointStamped pointStampedMsg;
  pointStampedMsg.header.stamp = ros::Time::now();
  pointStampedMsg.header.frame_id = "world"; // Set your desired frame_id

  // Assign Eigen vector components to PointStamped message
  pointStampedMsg.point.x = pathPoint(0);
  pointStampedMsg.point.y = pathPoint(1);
  pointStampedMsg.point.z = pathPoint(2);

  // Publish the PointStamped message
  pointPub.publish(pointStampedMsg);
}

//--------------------------------------------------------

ITaskBase::ITaskBase(ros::NodeHandle& n, double freq) : nh_(n), rosFreq_(freq), loopRate_(freq) {
  internalFSM_ = make_unique<msm::back::state_machine<TaskSafeFSM>>();

  // Initialize and test the FSM
  internalFSM_->start();
  internalFSM_->process_event(Initialized());
  internalFSM_->process_event(PathComputed());
  internalFSM_->process_event(SafetyTrigger());
  internalFSM_->process_event(Recover());
  internalFSM_->process_event(Start());
  internalFSM_->process_event(Finished());
  internalFSM_->process_event(SafetyTrigger());
  internalFSM_->process_event(Recover());

  // Create an unique pointer for the instance of DynamicalSystem
  dynamicalSystem_ = make_unique<DynamicalSystem>(rosFreq_);

  // Create an unique pointer for the instance of TargetExtraction
  targetExtraction_ = make_unique<TargetExtraction>(nh_);

  // Create an unique pointer for the instance of PathPlanner
  pathPlanner_ = make_unique<PathPlanner>(nh_);

  // Create an unique pointer for the instance of PathPlanner
  boustrophedonServer_ = make_unique<BoustrophedonServer>(nh_);

  //TODO(Tristan): delete rviz dependency
  pointPub_ = nh_.advertise<geometry_msgs::PointStamped>("path_point", 1);
  desiredVelFilteredPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  //------------------------------
}

bool ITaskBase::initialize(std::string robotName) {
  cout << "initialization shotcrete ..." << endl;

  if (robotName == "Ur5") {
    roboticArm_ = make_unique<RoboticArmUr5>();
    if (roboticArm_) {
      cout << "----------------------Ur5 chosen and well initializate----------------------------------" << endl;
      checkInitialization = true;
      homeJoint_ = roboticArm_->originalHomeJoint;
    } else {
      cout << "Error: roboticArm_ is null." << endl;
    }
  } else if (robotName == "Iiwa7") {
    roboticArm_ = make_unique<RoboticArmIiwa7>();
    if (roboticArm_) {
      cout << "----------------------Iiwa7 chosen and well initializate----------------------------------" << endl;
      checkInitialization = true;
      homeJoint_ = roboticArm_->originalHomeJoint;
    } else {
      cout << "Error: roboticArm_ is null." << endl;
    }
  } else {
    cout << "Please define a valid robot to perform shotcrete" << endl;
  }
  return checkInitialization;
}

bool ITaskBase::computePath() {
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
    loopRate_.sleep();
  }
  cout << "path well compute" << endl;
  return checkPath;
}

bool ITaskBase::goWorkingPosition() {

  vector<double> firstQuatPos = dynamicalSystem_->getFirstQuatPos();

  cout << "Go to first position :" << firstQuatPos[4] << firstQuatPos[5] << firstQuatPos[6] << endl;

  while (ros::ok() && !checkWorkingPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(firstQuatPos);
    checkWorkingPosition = dynamicalSystem_->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJoint = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);
    rosInterface_->sendState(desiredJoint);

    ros::spinOnce();
    loopRate_.sleep();

    //TODO(Tristan): delete rviz dependency
    twistMarker(twistDesiredEigen, pairActualQuatPos.second, desiredVelFilteredPub_);
  }
  return checkWorkingPosition;
}

bool ITaskBase::execute() {
  cout << "preforming shotcrete ..." << endl;

  while (ros::ok() && !checkFinish) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getDsQuatSpeed();

    checkFinish = dynamicalSystem_->finish;

    VectorXd twistDesiredEigen = roboticArm_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);

    rosInterface_->sendState(desiredJointSpeed);

    ros::spinOnce();
    loopRate_.sleep();
  }
  return checkFinish;
}

void ITaskBase::setHomingPosition(vector<double> desiredJoint) { homeJoint_ = desiredJoint; }

bool ITaskBase::goHomingPosition() {
  dynamicalSystem_->init = false;
  cout << "Go Home..." << endl;
  // get home position
  pair<Quaterniond, Vector3d> pairHomeQuatPos = roboticArm_->getFK(homeJoint_);
  Quaterniond homeQuat = pairHomeQuatPos.first;
  Vector3d homePos = pairHomeQuatPos.second;
  vector<double> desiredQuatPos = {
      homeQuat.x(), homeQuat.y(), homeQuat.z(), homeQuat.w(), homePos(0), homePos(1), homePos(2)};

  while (ros::ok() && !checkHomingPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(desiredQuatPos);
    checkHomingPosition = dynamicalSystem_->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJointSpeed = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);
    rosInterface_->sendState(desiredJointSpeed);

    ros::spinOnce();
    loopRate_.sleep();
  }

  return checkHomingPosition;
}
