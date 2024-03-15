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
void twistMarker(VectorXd twistDesiredEigen, Vector3d pos, ros::Publisher& marker_pub) {
  std::array<double, 3> scale_linear = {0.01, 0.1, 0.5};
  std::array<double, 4> color_linear = {1.0, 1.0, 0.0, 1.0}; // rgba, don't forget to set alpha
  std::array<double, 3> color_angular = {1.0, 0.0, 0.0};     // rgb, alpha is taken from linear

  visualization_msgs::Marker linear_marker, angular_marker;

  // Linear twist arrow marker
  linear_marker.header.frame_id = "iiwa_link_0"; // Set your desired frame ID
  linear_marker.header.stamp = ros::Time();
  linear_marker.ns = "twist";
  linear_marker.id = 0;
  linear_marker.type = visualization_msgs::Marker::ARROW;
  linear_marker.action = visualization_msgs::Marker::ADD;
  linear_marker.color.r = color_linear[0];
  linear_marker.color.g = color_linear[1];
  linear_marker.color.b = color_linear[2];
  linear_marker.color.a = color_linear[3];

  linear_marker.scale.x = scale_linear[0]; // Arrow width
  linear_marker.scale.y = scale_linear[1]; // Arrow head width
  linear_marker.scale.z = scale_linear[2]; // Arrow head length

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
  angular_marker.id += 1;
  angular_marker.color.r = color_linear[0];
  angular_marker.color.g = color_linear[1];
  angular_marker.color.b = color_linear[2];

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

ITaskBase::ITaskBase(ros::NodeHandle& n, double freq) : nh(n), rosFreq(freq), loopRate(freq) {
  // Create an unique pointer for the instance of DynamicalSystem
  dynamicalSystem = make_unique<DynamicalSystem>(rosFreq);
  // Create an unique pointer for the instance of TargetExtraction
  targetextraction = make_unique<TargetExtraction>(nh);
  // Create an unique pointer for the instance of PathPlanner
  pathplanner = make_unique<PathPlanner>(nh);
  // Create an unique pointer for the instance of PathPlanner
  boustrophedonserver = make_unique<BoustrophedonServer>(nh);

  //TODO(Tristan): delete rviz dependency
  pointPub = nh.advertise<geometry_msgs::PointStamped>("path_point", 1);
  desiredVelFilteredPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  //------------------------------
}

bool ITaskBase::initialize(std::string robotName) {
  cout << "initialization shotcrete ..." << endl;

  if (robotName == "Ur5") {
    roboticArm = make_unique<RoboticArmUr5>();
    if (roboticArm) {
      cout << "----------------------Ur5 chosen and well initializate----------------------------------" << endl;
      checkInit = true;
      homeJoint = roboticArm->originalHomeJoint;
    } else {
      cout << "Error: roboticArm is null." << endl;
    }
  } else if (robotName == "Iiwa7") {
    roboticArm = make_unique<RoboticArmIiwa7>();
    if (roboticArm) {
      cout << "----------------------Iiwa7 chosen and well initializate----------------------------------" << endl;
      checkInit = true;
      homeJoint = roboticArm->originalHomeJoint;
    } else {
      cout << "Error: roboticArm is null." << endl;
    }
  } else {
    cout << "Please define a valid robot to perform shotcrete" << endl;
  }
  return checkInit;
}

bool ITaskBase::computePath() {
  cout << "computing path ..." << endl;

  // extract polygons for boustrophedon
  // WARNING: need the position of the target from Optitrack to continue
  vector<Vector3d> polygons_positions = targetextraction->get_polygons();
  Quaterniond quatTarget = targetextraction->get_quat_target();
  Vector3d posTarget = targetextraction->get_pos_target();
  targetextraction->see_target();

  // initialization
  pathplanner->setTarget(quatTarget, posTarget, polygons_positions);
  double optimum_radius = pathplanner->getOptimumRadius();

  boustrophedonserver->setOptimumRad(optimum_radius);
  // wait for the action server to startnew_rad

  cout << "Waiting for action server to start." << endl;
  boustrophedonserver->initRosLaunch();

  cout << "Action server started" << endl;

  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal = pathplanner->ComputeGoal();
  boustrophedonserver->polygon_pub.publish(goal.property);

  cout << "Waiting for goal" << endl;

  nav_msgs::Path path;
  nav_msgs::Path path_transformed;

  while (ros::ok() && !checkPath) {
    ros::Time start_time = ros::Time::now();
    pathplanner->publishInitialPose();
    goal.robot_position = pathplanner->getInitialPose();
    boustrophedonserver->start_pub.publish(goal.robot_position);
    boustrophedonserver->client.sendGoal(goal);
    ROS_INFO_STREAM("Sending goal");

    // wait for the action to return
    bool finished_before_timeout = boustrophedonserver->client.waitForResult(ros::Duration(30.0));
    actionlib::SimpleClientGoalState state = boustrophedonserver->client.getState();
    boustrophedon_msgs::PlanMowingPathResultConstPtr result = boustrophedonserver->client.getResult();
    if (result->plan.points.size() < 1) {
      ROS_INFO("Action did not finish before the time out.");
    } else {
      ROS_INFO("Action finished: %s", state.toString().c_str());

      cout << "Result with : " << result->plan.points.size() << endl;

      if (result->plan.points.size() > 2) {
        pathplanner->convertStripingPlanToPath(result->plan, path);

        path_transformed = pathplanner->get_transformed_path(path);

        vector<vector<double>> vectorPathTransformed = pathplanner->convertPathPlanToVectorVector(path_transformed);

        vector<double> firstQuatPos = vectorPathTransformed[0];

        dynamicalSystem->set_path(vectorPathTransformed);

        boustrophedonserver->path_pub.publish(path_transformed);

        boustrophedonserver->closeRosLaunch();
        checkPath = true;
      }
    }
    ros::spinOnce();
    loopRate.sleep();
  }
  cout << "path well compute" << endl;
  return checkPath;
}

bool ITaskBase::goWorkingPosition() {

  vector<double> firstQuatPos = dynamicalSystem->getFirstQuatPos();

  cout << "Go to first position :" << firstQuatPos[4] << firstQuatPos[5] << firstQuatPos[6] << endl;

  while (ros::ok() && !checkWorkingPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getLinearDsOnePosition(firstQuatPos);
    checkWorkingPosition = dynamicalSystem->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJoint = roboticArm->low_level_controller(stateJoints, twistDesiredEigen);
    rosInterface->send_state(desiredJoint);

    ros::spinOnce();
    loopRate.sleep();

    //TODO(Tristan): delete rviz dependency
    twistMarker(twistDesiredEigen, pairActualQuatPos.second, desiredVelFilteredPub);
  }
  return checkWorkingPosition;
}

bool ITaskBase::execute() {
  cout << "preforming shotcrete ..." << endl;

  while (ros::ok() && !checkFinish) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->get_DS_quat_speed();

    checkFinish = dynamicalSystem->finish;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm->low_level_controller(stateJoints, twistDesiredEigen);

    rosInterface->send_state(desiredJointSpeed);

    ros::spinOnce();
    loopRate.sleep();
  }
  return checkFinish;
}

void ITaskBase::setHomingPosition(vector<double> desiredJoint) { homeJoint = desiredJoint; }

bool ITaskBase::goHomingPosition() {
  cout << "Go Home..." << endl;
  // get home position
  pair<Quaterniond, Vector3d> pairHomeQuatPos = roboticArm->getFK(homeJoint);
  Quaterniond homeQuat = pairHomeQuatPos.first;
  Vector3d homePos = pairHomeQuatPos.second;
  vector<double> desiredQuatPos = {
      homeQuat.x(), homeQuat.y(), homeQuat.z(), homeQuat.w(), homePos(0), homePos(1), homePos(2)};

  while (ros::ok() && !checkHomingPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getLinearDsOnePosition(desiredQuatPos);
    checkHomingPosition = dynamicalSystem->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJointSpeed = roboticArm->low_level_controller(stateJoints, twistDesiredEigen);
    rosInterface->send_state(desiredJointSpeed);

    ros::spinOnce();
    loopRate.sleep();
  }

  return checkHomingPosition;
}
