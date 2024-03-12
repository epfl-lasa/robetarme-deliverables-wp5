#include "Tasks.h"
#include <yaml-cpp/yaml.h>
// include all the robot needeed
#include "RoboticArmUr5.h"


using namespace std;
using namespace Eigen;

Tasks::Tasks(ros::NodeHandle& n, double freq) : nh(n), rosFreq(freq),loop_rate(freq) {
  // Create an unique pointer for the instance of DynamicalSystem
  dynamicalSystem = make_unique<DynamicalSystem>(rosFreq);
  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface = make_unique<RosInterfaceNoetic>(nh);
  // Create an unique pointer for the instance of TargetExtraction
  targetextraction = make_unique<TargetExtraction>(nh);
  // Create an unique pointer for the instance of PathPlanner
  pathplanner = make_unique<PathPlanner>(nh);
  // Create an unique pointer for the instance of PathPlanner
  boustrophedonserver = make_unique<BoustrophedonServer>(nh);
}

bool Tasks::initShotcrete(){
  cout<< "initialization shotcrete ..."<< endl;
  string yaml_path = string(WP5_TASKS_DIR) + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);

  // Access parameters from the YAML file
  string robotName = config["shotcrete"]["robot_name"].as<string>();
  if(robotName  == "Ur5"){  
    roboticArm = make_unique<RoboticArmUr5>();
    if (roboticArm) {
      cout<< "Ur5 chosen and well initializate"<< endl;
      checkInit = true;
      homeJoint =  roboticArm->originalHomeJoint;
    } 
    else {
      cout << "Error: roboticArm is null." << endl;
    }
  }
  else{
    cout<< "Please define a valid robot to perform shotcrete"<< endl;
  }
  return  checkInit;
}

bool Tasks::computePathShotcrete(){
  cout<< "computing path ..."<< endl;

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

  while (ros::ok() &&  !checkPath) {
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
    cout<< "path well compute" << endl;
    return checkPath;
}

bool Tasks::goFirstPosition(){
  cout<< "Go first position ..."<< endl;

  vector<double> firstQuatPos = dynamicalSystem->getFirstQuatPos();

  while (ros::ok()  && !checkFirstPosition) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getLinearDsOnePosition(firstQuatPos);
    checkFirstPosition =  dynamicalSystem->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm->getIDynamics(actualJoint, twistDesiredEigen);
    rosInterface->send_state(desiredJointSpeed);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return checkFirstPosition;
}

bool Tasks::DoShotcrete(){
    cout<< "preforming shotcrete ..."<< endl;

    while (ros::ok() && !checkFinish) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->get_DS_quat_speed();

    checkFinish =  dynamicalSystem->finish;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm->getIDynamics(actualJoint, twistDesiredEigen);

    rosInterface->send_state(desiredJointSpeed);

    ros::spinOnce();
    loop_rate.sleep();
  }
    return checkFinish;
}

void Tasks::setHome(  vector<double> desiredJoint){
  homeJoint = desiredJoint;
}

bool Tasks::goHome(){
  cout<< "Go Home..."<< endl;
  // get home position
  pair<Quaterniond, Vector3d> pairHomeQuatPos = roboticArm->getFK(homeJoint);
  Quaterniond homeQuat= pairHomeQuatPos.first;
  Vector3d homePos= pairHomeQuatPos.second;
  vector<double> desiredQuatPos= {homeQuat.x(),homeQuat.y(),homeQuat.z(),homeQuat.w(),homePos(0),homePos(1),homePos(2)};

  while (ros::ok()  && !checkGoHome) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getLinearDsOnePosition(desiredQuatPos);
    checkGoHome =  dynamicalSystem->checkLinearDs;

    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm->getIDynamics(actualJoint, twistDesiredEigen);
    rosInterface->send_state(desiredJointSpeed);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return checkGoHome;
}

