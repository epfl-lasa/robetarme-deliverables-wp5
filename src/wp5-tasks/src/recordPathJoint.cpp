#include "TaskShotcrete.h"

using namespace std;
using namespace Eigen;

TaskShotcrete::TaskShotcrete(ros::NodeHandle& nh, double freq, string robotName) : ITaskBase(nh, freq, robotName) {
  ros::NodeHandle nodeHandle = getRosNodehandle_();

  // // Create an unique pointer for the instance of TargetExtraction
  // targetExtraction_ = make_unique<TargetExtraction>(nodeHandle);

  // // Create an unique pointer for the instance of PathPlanner
  // pathPlanner_ = make_unique<PathPlanner>(nodeHandle);

  // Instantiate the objects
  polygonCoverage_ = std::make_unique<PolygonCoverage>(nodeHandle);

  // Create an unique pointer for the instance of Tool
  tools_ = make_unique<ToolsShotcrete>();

  takeConfigTask("shotcrete");
  dynamicalSystem_->setOffset(tools_->getOffset());
  dynamicalSystem_->setLimitCycleSpeedConv(limitCycleSpeed_, convRate_);
  dynamicalSystem_->setLinearSpeed(linearSpeed_);
}

bool TaskShotcrete::computePath() {
  string alternativeYamlPath = string(WP5_TASKS_DIR) + "/txts/path_calibration.txt";

  // Open the output file in write mode
  ofstream outputFile(alternativeYamlPath);

  // Check if the file is open
  if (!outputFile.is_open()) {
    cerr << "Failed to open the file." << endl;
    return 1;
  }

  // Wait for Enter key press instead of sleep(5)
  cout << "Press Enter to start...";
  cin.get(); // Waits for Enter key press
  while (ros::ok()) {
    ros::spinOnce();
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);

    // Write the values to the file in 6 columns

    outputFile << actualJoint[0] << " " << actualJoint[1] << " " << actualJoint[2] << " " << actualJoint[3] << " "
               << actualJoint[4] << " " << actualJoint[5] << endl;
    ros::Duration(2).sleep(); // Sleep for 5 seconds
    cout << "record joint state in 3 sec " << endl;
    ros::Duration(3).sleep(); // Sleep for 5 seconds
  }
  // Close the output file
  outputFile.close();

  return false;

  //------------------------------------------------------------------------------------------------
  bool checkPath = false;

  //TODO: understand why checkpython is false
  cout << "get Pointcloud.. " << endl;

  polygonCoverage_->getPointCloud();
  ros::spinOnce();
  getRosLoopRate_()->sleep();

  cout << "initializazion pathplanner:" << endl;
  cout << "step 1 : featurespace" << endl;

  //TODO: change name of the functino and fill
  //MAKE GRID
  // trasnform the point cloud to the featur space
  polygonCoverage_->makeMesh();
  //GETUVMAP
  polygonCoverage_->makeUVmap();

  cout << "step 2: convert polygon from pointcloud" << endl;

  // it will save the polygons to the .txt file
  polygonCoverage_->convertPclToPolygon();

  cout << "step 3: boustrophedon in feature space" << endl;

  // Read the polygon from the txt file
  vector<Vector3d> polygonsPositions = polygonCoverage_->getFlatPolygonFromTxt();

  // Simplify the polygon
  double epsilon = 0.2; // Tolerance for simplification
  vector<Vector3d> simplifiedPolygon = polygonCoverage_->rdp(polygonsPositions, epsilon);

  polygonCoverage_->seePolygonFlat(simplifiedPolygon);

  //start boustrophedon aogirthm
  polygonCoverage_->initRosLaunch();

  //set target for boustrophedon
  vector<Vector3d> holl_points;
  polygonCoverage_->callSetPolygonService(simplifiedPolygon, holl_points);

  //set start and finish point for boustrophedon
  polygonCoverage_->callStartService(simplifiedPolygon[0], simplifiedPolygon[0]);
  string name = "waypointInFeatureSpace";
  while (ros::ok() && !polygonCoverage_->checkPathReceived()) {
    ros::spinOnce();
    getRosLoopRate_()->sleep();
  }
  polygonCoverage_->writePathToFile(polygonCoverage_->getPathFromPolygonFlat(), name);

  polygonCoverage_->closeRosLaunch();

  cout << "step 4: boustrophedon in original space" << endl;

  // send the path from feature space to hae it on realspace
  polygonCoverage_->getPathFromFeatureSpaceToRealSpace();

  // read the .txt with the point of the path to follows
  nav_msgs::Path pathTransformed;
  pathTransformed = polygonCoverage_->convertFileToNavMsgsPath();

  polygonCoverage_->publishNavmsg(pathTransformed);

  ros::spinOnce();
  getRosLoopRate_()->sleep();

  vector<vector<double>> vectorPathTransformed = polygonCoverage_->convertNavPathToVectorVector(pathTransformed);

  //TODO: check if the path is well computed
  cout << "path well compute" << endl;
  dynamicalSystem_->setPath(vectorPathTransformed);
  return checkPath = true;
}

bool TaskShotcrete::execute() {
  cout << "preforming shotcrete ..." << endl;
  dynamicalSystem_->resetInit();

  while (ros::ok() && !dynamicalSystem_->isFinished()) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface_->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    vector<double> actualJointSpeed = get<1>(stateJoints);

    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm_->getFK(actualJoint);

    dynamicalSystem_->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem_->getDsQuatSpeed();

    VectorXd twistDesiredEigen = dynamicalSystem_->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm_->lowLevelController(stateJoints, twistDesiredEigen);

    rosInterface_->sendState(desiredJointSpeed);

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

    rosInterface_->setCartesianPose(pairActualQuatPos);
    rosInterface_->setCartesianTwist(actualTwist);
    rosInterface_->setDesiredDsTwist(desiredTwist);

    ros::spinOnce();
    getRosLoopRate_()->sleep();
  }

  return dynamicalSystem_->isFinished();
}
