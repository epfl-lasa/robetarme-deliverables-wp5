
// clang- format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PolygonCoverage.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TaskFSM.h"
#include "TaskFactory.h"
#include "TaskShotcrete.h"
#include "TaskSurfaceFinishing.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  double rosFreq = 30;
  string robotName = "ur5_robot";
  bool checkPath = false;

  // Init ros
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nodeHandle;
  ros::Rate loopRate(rosFreq);

  // Create unique pointers for the instances
  std::unique_ptr<PolygonCoverage> polygonCoverage = nullptr;

  // Instantiate the objects
  polygonCoverage = std::make_unique<PolygonCoverage>(nodeHandle);

  //TODO: understand why checkpython is false
  cout << "transformation of the poinctloud to the robot frame ..." << endl;
  bool checkpython = polygonCoverage->pointCloudTransformer();

  cout << "initializazion pathplanner:" << endl;
  cout << "step 1 : featurespace" << endl;

  //TODO: change name of the functino and fill
  //MAKE GRID
  // trasnform the point cloud to the featur space
  polygonCoverage->makeMesh();
  //GETUVMAP
  polygonCoverage->makeUVmap();

  cout << "step 2: convert polygon from pointcloud" << endl;

  // it will save the polygons to the .txt file
  polygonCoverage->convertPclToPolygon();

  cout << "step 3: boustrophedon in feature space" << endl;

  // Read the polygon from the txt file
  vector<Vector3d> polygonsPositions = polygonCoverage->getFlatPolygonFromTxt();

  // Simplify the polygon
  double epsilon = 0.2; // Tolerance for simplification
  vector<Vector3d> simplifiedPolygon = polygonCoverage->rdp(polygonsPositions, epsilon);

  polygonCoverage->seePolygonFlat(simplifiedPolygon);

  //start boustrophedon aogirthm
  polygonCoverage->initRosLaunch();

  //set target for boustrophedon
  vector<Vector3d> holl_points;
  polygonCoverage->callSetPolygonService(simplifiedPolygon, holl_points);

  //set start and finish point for boustrophedon
  polygonCoverage->callStartService(simplifiedPolygon[0], simplifiedPolygon[0]);
  string name = "waypointInFeatureSpace";
  while (ros::ok() && !polygonCoverage->checkPathReceived()) {
    ros::spinOnce();
    loopRate.sleep();
  }
  polygonCoverage->writePathToFile(polygonCoverage->getPathFromPolygonFlat(), name);

  polygonCoverage->closeRosLaunch();

  cout << "step 4: boustrophedon in original space" << endl;

  // send the path from feature space to hae it on realspace
  polygonCoverage->getPathFromFeatureSpaceToRealSpace();

  // read the .txt with the point of the path to follows
  nav_msgs::Path pathTransformed;
  pathTransformed = polygonCoverage->convertFileToNavMsgsPath();

  polygonCoverage->publishNavmsg(pathTransformed);

  ros::spinOnce();
  loopRate.sleep();

  vector<vector<double>> vectorPathTransformed = polygonCoverage->convertNavPathToVectorVector(pathTransformed);

  //TODO: check if the path is well computed
  cout << "path well compute" << endl;
  // dynamicalSystem_->setPath(vectorPathTransformed);

  return 0;
}
