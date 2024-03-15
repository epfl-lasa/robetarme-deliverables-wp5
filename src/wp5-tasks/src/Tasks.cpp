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

Tasks::Tasks(ros::NodeHandle& n, double freq) : nh(n), rosFreq(freq), loop_rate(freq) {
  // Create an unique pointer for the instance of DynamicalSystem
  dynamicalSystem = make_unique<DynamicalSystem>(rosFreq);
  // Create an unique pointer for the instance of TargetExtraction
  targetextraction = make_unique<TargetExtraction>(nh);
  // Create an unique pointer for the instance of PathPlanner
  pathplanner = make_unique<PathPlanner>(nh);
  // Create an unique pointer for the instance of PathPlanner
  boustrophedonserver = make_unique<BoustrophedonServer>(nh);

  //TODO: delet rviz dependency
  point_pub = nh.advertise<geometry_msgs::PointStamped>("path_point", 1);
  pub_desired_vel_filtered = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  //------------------------------
}

bool Tasks::initShotcrete() {
  cout << "initialization shotcrete ..." << endl;
  string yaml_path = string(WP5_TASKS_DIR) + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);

  // Access parameters from the YAML file
  string robotName = config["shotcrete"]["robot_name"].as<string>();

  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface = make_unique<RosInterfaceNoetic>(nh, robotName);

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

bool Tasks::computePathShotcrete() {
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
    loop_rate.sleep();
  }
  cout << "path well compute" << endl;
  return checkPath;
}

bool Tasks::goFirstPosition() {

  vector<double> firstQuatPos = dynamicalSystem->getFirstQuatPos();

  cout << "Go to first position, pointing on the target point:" << firstQuatPos[4] << firstQuatPos[5] << firstQuatPos[6]
       << endl;

  dynamicalSystem->init = false;

  while (ros::ok() && !checkFirstPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getLinearDsOnePosition(firstQuatPos);
    checkFirstPosition = dynamicalSystem->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJoint = roboticArm->low_level_controller(stateJoints, twistDesiredEigen);
    rosInterface->send_state(desiredJoint);

    ros::spinOnce();
    loop_rate.sleep();

    //TODO: delet rviz dependency
    twistMarker(twistDesiredEigen, pairActualQuatPos.second, pub_desired_vel_filtered);
  }
  return checkFirstPosition;
}

bool Tasks::DoShotcrete() {
  cout << "preforming shotcrete ..." << endl;
  dynamicalSystem->init = false;

  while (ros::ok() && !checkFinish) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getDsQuatSpeed();

    checkFinish = dynamicalSystem->finish;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm->low_level_controller(stateJoints, twistDesiredEigen);

    rosInterface->send_state(desiredJointSpeed);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return checkFinish;
}

void Tasks::setHome(vector<double> desiredJoint) { homeJoint = desiredJoint; }

bool Tasks::goHome() {
  dynamicalSystem->init = false;
  cout << "Go Home..." << endl;
  // get home position
  pair<Quaterniond, Vector3d> pairHomeQuatPos = roboticArm->getFK(homeJoint);
  Quaterniond homeQuat = pairHomeQuatPos.first;
  Vector3d homePos = pairHomeQuatPos.second;
  vector<double> desiredQuatPos =
      {homeQuat.x(), homeQuat.y(), homeQuat.z(), homeQuat.w(), homePos(0), homePos(1), homePos(2)};

  while (ros::ok() && !checkGoHome) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getLinearDsOnePosition(desiredQuatPos);
    checkGoHome = dynamicalSystem->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJointSpeed = roboticArm->low_level_controller(stateJoints, twistDesiredEigen);
    rosInterface->send_state(desiredJointSpeed);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return checkGoHome;
}
