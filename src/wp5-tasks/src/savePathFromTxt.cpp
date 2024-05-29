
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
  cout << "transformation check:" << endl;
  cout << checkpython << endl;

  cout << "computing path ..." << endl;

  //TODO: send the pointcloud transformer to python code from RUI
  // it will save the polygons to the .txt file

  // Read the polygon from the txt file
  vector<Vector3d> polygonsPositions = polygonCoverage->readFlatPolygonFromTxt();

  // Simplify the polygon
  double epsilon = 0.2; // Tolerance for simplification
  vector<Vector3d> simplifiedPolygon = polygonCoverage->rdp(polygonsPositions, epsilon);

  // Output the read points for verification
  for (const auto& point : simplifiedPolygon) {
    cout << point.transpose() << endl;
  }

  polygonCoverage->seePolygonFlat(simplifiedPolygon);

  cout << "Waiting for action server to start." << endl;
  polygonCoverage->initRosLaunch();

  vector<Vector3d> holl_points;
  polygonCoverage->callSetPolygonService(simplifiedPolygon, holl_points);

  cout << "Waiting for goal" << endl;
  polygonCoverage->callStartService(simplifiedPolygon[0], simplifiedPolygon[0]);

  // nav_msgs::Path path;
  nav_msgs::Path pathTransformed;
  pathTransformed = polygonCoverage->convertFileToNavMsgsPath();

  polygonCoverage->publishNavmsg(pathTransformed);

  ros::spinOnce();
  loopRate.sleep();

  polygonCoverage->closeRosLaunch();

  vector<vector<double>> vectorPathTransformed = polygonCoverage->convertNavPathToVectorVector(pathTransformed);

  //TODO: check if the path is well computed
  cout << "path well compute" << endl;
  // dynamicalSystem_->setPath(vectorPathTransformed);

  string file_path_flat = string(WP5_TASKS_DIR) + "/txts/path_flat.txt";
  string file_path_original = string(WP5_TASKS_DIR) + "/txts/path_original.txt";

  polygonCoverage->writePathToFile(pathTransformed, file_path_original);
  polygonCoverage->writePathToFile(polygonCoverage->getPathFromPolygonFlat(), file_path_flat);

  return 0;
}
