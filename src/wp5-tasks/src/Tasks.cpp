#include "Tasks.h"
#include <yaml-cpp/yaml.h>
// include all the robot needeed
#include "RoboticArmIiwa7.h"
#include "RoboticArmUr5.h"

//TODO: remove rviz dependency
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/Point.h>        //<----------to remove
#include <geometry_msgs/PointStamped.h> //<----------to remove

using namespace std;
using namespace Eigen;

//TODO:reviz dep

//TODO:RVIZ DEP
void twistMarker(VectorXd twistDesiredEigen, Vector3d pos, ros::Publisher& marker_pub) {
  visualization_msgs::Marker linear_marker, angular_marker;

  // Linear twist arrow marker
  linear_marker.header.frame_id = "iiwa_link_0"; // Set your desired frame ID
  linear_marker.header.stamp = ros::Time();
  linear_marker.ns = "twist";
  linear_marker.id = 0;
  linear_marker.type = visualization_msgs::Marker::ARROW;
  linear_marker.action = visualization_msgs::Marker::ADD;
  linear_marker.color.r = 1.0;
  linear_marker.color.g = 1.0;
  linear_marker.color.b = 0.0;
  linear_marker.color.a = 1.0; // Don't forget to set the alpha!

  linear_marker.scale.x = 0.01; // Arrow width
  linear_marker.scale.y = 0.1;  // Arrow head width
  linear_marker.scale.z = 0.5;  // Arrow head length

  linear_marker.pose.orientation.w = 1.0;
  linear_marker.pose.position.x = pos(0);
  linear_marker.pose.position.y = pos(1);
  linear_marker.pose.position.z = pos(2);

  linear_marker.points.push_back(geometry_msgs::Point());
  geometry_msgs::Point point;
  point.x = twistDesiredEigen(3);
  point.y = twistDesiredEigen(4);
  point.z = twistDesiredEigen(5);
  linear_marker.points.push_back(point);

  // Angular twist arrow marker
  angular_marker = linear_marker; // Copy settings from linear_marker
  angular_marker.id = 1;
  angular_marker.color.r = 1.0;
  angular_marker.color.g = 0.0;
  angular_marker.color.b = 0.0;

  // Angular twist direction
  angular_marker.points.push_back(geometry_msgs::Point());
  point.x = twistDesiredEigen(0);
  point.y = twistDesiredEigen(1);
  point.z = twistDesiredEigen(2);
  angular_marker.points.push_back(point);

  // Publish markers
  marker_pub.publish(linear_marker);
  //marker_pub.publish(angular_marker);
}

void publishPointStamped(const Vector3d& pathPoint, ros::Publisher pointPub) {
  geometry_msgs::PointStamped point_stamped_msg;
  point_stamped_msg.header.stamp = ros::Time::now();
  point_stamped_msg.header.frame_id = "world"; // Set your desired frame_id

  // Assign Eigen vector components to PointStamped message
  point_stamped_msg.point.x = pathPoint(0);
  point_stamped_msg.point.y = pathPoint(1);
  point_stamped_msg.point.z = pathPoint(2);

  // Publish the PointStamped message
  pointPub.publish(point_stamped_msg);
}

//--------------------------------------------------------

Tasks::Tasks(ros::NodeHandle& n, double freq) : nh_(n), rosFreq_(freq), loopRate_(freq) {
  // Create an unique pointer for the instance of DynamicalSystem
  dynamicalSystem_ = make_unique<DynamicalSystem>(rosFreq_);
  // Create an unique pointer for the instance of TargetExtraction
  targetextraction_ = make_unique<TargetExtraction>(nh_);
  // Create an unique pointer for the instance of PathPlanner
  pathplanner_ = make_unique<PathPlanner>(nh_);
  // Create an unique pointer for the instance of PathPlanner
  boustrophedonserver_ = make_unique<BoustrophedonServer>(nh_);

  //TODO: delet rviz dependency
  pointPub_ = nh_.advertise<geometry_msgs::PointStamped>("path_point", 1);
  pubDesiredVelFiltered_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  //------------------------------

  outputTwist_ = Eigen::VectorXd(6);
}

bool Tasks::initTask(string taskName) {
  cout << "initialization " << taskName << " ..." << endl;
  string yamlPath = string(WP5_TASKS_DIR) + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);

  // Access parameters from the YAML file
  string robotName = config[taskName]["robot_name"].as<string>();

  if (robotName == "Ur5") {
    roboticArm_ = make_unique<RoboticArmUr5>();
    if (roboticArm_) {
      cout << "----------------------Ur5 chosen and well initializate----------------------------------" << endl;
      checkInit = true;
      homeJoint_ = roboticArm_->originalHomeJoint;
    } else {
      cout << "Error: roboticArm_ is null." << endl;
    }
  } else if (robotName == "Iiwa7") {
    roboticArm_ = make_unique<RoboticArmIiwa7>();
    if (roboticArm_) {
      cout << "----------------------Iiwa7 chosen and well initializate----------------------------------" << endl;
      checkInit = true;
      homeJoint_ = roboticArm_->originalHomeJoint;
    } else {
      cout << "Error: roboticArm_ is null." << endl;
    }
  } else {
    cout << "Please define a valid robot to perform shotcrete" << endl;
  }
  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface_ = make_unique<RosInterfaceNoetic>(nh_, robotName);
  return checkInit;
}

bool Tasks::computePathShotcrete() {
  cout << "computing path ..." << endl;

  // extract polygons for boustrophedon
  // WARNING: need the position of the target from Optitrack to continue
  vector<Vector3d> polygons_positions = targetextraction_->getPolygons();
  Quaterniond quatTarget = targetextraction_->getQuatTarget();
  Vector3d posTarget = targetextraction_->getPosTarget();
  targetextraction_->seeTarget();

  // initialization
  pathplanner_->setTarget(quatTarget, posTarget, polygons_positions);
  double optimumRadius = pathplanner_->getOptimumRadius();

  boustrophedonserver_->setOptimumRad(optimumRadius);
  // wait for the action server to startnew_rad

  cout << "Waiting for action server to start." << endl;
  boustrophedonserver_->initRosLaunch();

  cout << "Action server started" << endl;

  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal = pathplanner_->ComputeGoal();
  boustrophedonserver_->polygonPub.publish(goal.property);

  cout << "Waiting for goal" << endl;

  nav_msgs::Path path;
  nav_msgs::Path path_transformed;

  while (ros::ok() && !checkPath) {
    ros::Time start_time = ros::Time::now();
    pathplanner_->publishInitialPose();
    goal.robot_position = pathplanner_->getInitialPose();
    boustrophedonserver_->startPub.publish(goal.robot_position);
    boustrophedonserver_->client.sendGoal(goal);
    ROS_INFO_STREAM("Sending goal");

    // wait for the action to return
    bool finished_before_timeout = boustrophedonserver_->client.waitForResult(ros::Duration(30.0));
    actionlib::SimpleClientGoalState state = boustrophedonserver_->client.getState();
    boustrophedon_msgs::PlanMowingPathResultConstPtr result = boustrophedonserver_->client.getResult();
    if (result->plan.points.size() < 1) {
      ROS_INFO("Action did not finish before the time out.");
    } else {
      ROS_INFO("Action finished: %s", state.toString().c_str());

      cout << "Result with : " << result->plan.points.size() << endl;

      if (result->plan.points.size() > 2) {
        pathplanner_->convertStripingPlanToPath(result->plan, path);

        path_transformed = pathplanner_->getTransformedPath(path);

        vector<vector<double>> vectorPathTransformed = pathplanner_->convertPathPlanToVectorVector(path_transformed);

        vector<double> firstQuatPos = vectorPathTransformed[0];

        dynamicalSystem_->set_path(vectorPathTransformed);

        boustrophedonserver_->pathPub.publish(path_transformed);

        boustrophedonserver_->closeRosLaunch();
        checkPath = true;
      }
    }
    ros::spinOnce();
    loopRate_.sleep();
  }
  cout << "path well compute" << endl;
  return checkPath;
}

bool Tasks::goFirstPosition() {
  vector<double> firstQuatPos = dynamicalSystem_->getFirstQuatPos();

  cout << "Go to first position, pointing on the target point:" << firstQuatPos[4] << firstQuatPos[5] << firstQuatPos[6]
       << endl;

  dynamicalSystem_->init = false;

  while (ros::ok() && !checkFirstPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(firstQuatPos);
    checkFirstPosition = dynamicalSystem_->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJoint = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);
    rosInterface_->sendState(desiredJoint);

    ros::spinOnce();
    loopRate_.sleep();

    //TODO: delet rviz dependency
    twistMarker(twistDesiredEigen, pairActualQuatPos.second, pubDesiredVelFiltered_);
  }
  return checkFirstPosition;
}

bool Tasks::DoShotcrete() {
  cout << "preforming shotcrete ..." << endl;
  dynamicalSystem_->init = false;

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

void Tasks::setHome(vector<double> desiredJoint) { homeJoint_ = desiredJoint; }

bool Tasks::goHome() {
  dynamicalSystem_->init = false;
  cout << "Go Home..." << endl;
  // get home position
  pair<Quaterniond, Vector3d> pairHomeQuatPos = roboticArm_->getFK(homeJoint_);
  Quaterniond homeQuat = pairHomeQuatPos.first;
  Vector3d homePos = pairHomeQuatPos.second;
  vector<double> desiredQuatPos =
      {homeQuat.x(), homeQuat.y(), homeQuat.z(), homeQuat.w(), homePos(0), homePos(1), homePos(2)};

  while (ros::ok() && !checkGoHome) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(desiredQuatPos);
    checkGoHome = dynamicalSystem_->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJointSpeed = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);
    rosInterface_->sendState(desiredJointSpeed);

    ros::spinOnce();
    loopRate_.sleep();
  }
  return checkGoHome;
}

void Tasks::set_bias() {
  int meanNum = 500;
  std::cout << "Recording F/T sensor bias. Please do not touch the robot for 3 seconds..." << std::endl;

  // Initialize wrenchActual and receivedWrench vectors
  std::vector<double> wrenchActual(6, 0.0); // Assuming 6 elements in the wrench
  std::vector<double> receivedWrench = rosInterface_->receiveWrench();

  int meanIteration = 0;
  while (ros::ok() && (meanIteration < meanNum)) {
    for (size_t i = 0; i < wrenchActual.size(); ++i) {
      wrenchActual[i] += receivedWrench[i] / meanNum;
    }
    meanIteration += 1;
  }

  // Assign the calculated bias to biasWrench_
  biasWrench_ = wrenchActual;

  // Print all elements of the biasWrench_ vector
  std::cout << "Recording F/T sensor bias done:" << std::endl;
  for (size_t i = 0; i < biasWrench_.size(); ++i) {
    std::cout << "biasWrench_[" << i << "] = " << biasWrench_[i] << std::endl;
  }
}

Eigen::VectorXd Tasks::decoderWrench() {
  vector<double> receivedWrench = rosInterface_->receiveWrench();
  Eigen::VectorXd outTwist(6);
  for (size_t i = 0; i < receivedWrench.size(); ++i) {
    receivedWrench[i] -= biasWrench_[i];
    if (receivedWrench[i] > 5) {
      outTwist(i) = -receivedWrench[i] * 0.01;
      if (outTwist(i) < -0.15) {
        outTwist(i) = -0.15;
      }
    } else if (receivedWrench[i] < -5) {
      outTwist(i) = receivedWrench[i] * 0.01;
      if (outTwist(i) > 0.15) {
        outTwist(i) = 0.15;
      }
    } else {
      outTwist(i) = 0;
    }
  }
  double alpha = 0.25;
  outputTwist_ = alpha * outputTwist_ + (1 - alpha) * outTwist;
  return outputTwist_;
}

bool Tasks::TestSF() {
  vector<double> firstQuatPos = dynamicalSystem_->getFirstQuatPos();
  cout << "Go to first position, pointing on the target point:" << firstQuatPos[4] << firstQuatPos[5] << firstQuatPos[6]
       << endl;

  dynamicalSystem_->init = false;

  while (ros::ok() && !checkFirstPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getLinearDsOnePosition(firstQuatPos);
    dynamicalSystem_->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    //TEST
    Eigen::VectorXd deltaTwist;
    deltaTwist = decoderWrench();
    cout << deltaTwist << endl;
    vector<double> desiredJoint = roboticArm_->lowLevelControllerSF(stateJoints, twistDesiredEigen, deltaTwist);
    //--------

    rosInterface_->sendState(desiredJoint);

    ros::spinOnce();
    loopRate_.sleep();

    //TODO: delet rviz dependency
    twistMarker(twistDesiredEigen, pairActualQuatPos.second, pubDesiredVelFiltered_);
  }
  return checkFirstPosition;
}
