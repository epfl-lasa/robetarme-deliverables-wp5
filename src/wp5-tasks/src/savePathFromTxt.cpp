
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

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "TaskFSM.h"
#include "TaskFactory.h"
#include "TaskShotcrete.h"
#include "TaskSurfaceFinishing.h"

using namespace std;
using namespace Eigen;

boustrophedon_msgs::PlanMowingPathGoal createPlanMowingPathGoal(const std::vector<Eigen::Vector3d>& points) {
  boustrophedon_msgs::PlanMowingPathGoal goal;

  goal.property.header.stamp = ros::Time::now();
  goal.property.header.frame_id = "base";
  goal.property.polygon.points.resize(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    goal.property.polygon.points[i].x = points[i][0];
    goal.property.polygon.points[i].y = points[i][1];
    goal.property.polygon.points[i].z = points[i][2];
  }

  goal.robot_position.pose.orientation.x = 0.0;
  goal.robot_position.pose.orientation.y = 0.0;
  goal.robot_position.pose.orientation.z = 0.0;
  goal.robot_position.pose.orientation.w = 1.0;

  return goal;
}

void printGoal(const boustrophedon_msgs::PlanMowingPathGoal& goal) {
  std::cout << "Goal points:" << std::endl;
  for (const auto& point : goal.property.polygon.points) {
    std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
  }
}

int main(int argc, char** argv) {
  double rosFreq = 300;
  string robotName = "ur5_robot";

  // Init ros
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nodeHandle;
  ros::Rate loopRate(rosFreq);

  // Create unique pointers for the instances
  std::unique_ptr<TargetExtraction> targetExtraction = nullptr;
  std::unique_ptr<PathPlanner> pathPlanner = nullptr;
  std::unique_ptr<BoustrophedonServer> boustrophedonServer = nullptr;

  // Instantiate the objects
  targetExtraction = std::make_unique<TargetExtraction>(nodeHandle);
  pathPlanner = std::make_unique<PathPlanner>(nodeHandle);
  boustrophedonServer = std::make_unique<BoustrophedonServer>(nodeHandle);

  cout << "computing path ..." << endl;

  // Extract polygons for boustrophedon
  std::ifstream inputFile(string(WP5_TASKS_DIR) + "/txts/boustrophedon_20points.txt");
  std::vector<Eigen::Vector3d> polygonsPositions;

  if (!inputFile.is_open()) {
    std::cerr << "Unable to open file" << std::endl;
    return 1;
  }

  int i_decrease = 0;
  double x, y, z;
  while (inputFile >> x >> y >> z) {
    if ((i_decrease % 15) == 0) {
      Eigen::Vector3d point(x, y, z);
      polygonsPositions.push_back(point);
    }
    i_decrease++;
  }
  inputFile.close();

  // Output the read points for verification
  for (const auto& point : polygonsPositions) {
    std::cout << point.transpose() << std::endl;
  }

  cout << "Waiting for action server to start." << endl;
  boustrophedonServer->initRosLaunch();
  cout << "Action server started" << endl;

  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal = createPlanMowingPathGoal(polygonsPositions);

  printGoal(goal);
  boustrophedonServer->polygonPub.publish(goal.property);

  cout << "Waiting for goal" << endl;

  nav_msgs::Path path;
  nav_msgs::Path pathTransformed;

  while (ros::ok()) {
    ros::Time start_time = ros::Time::now();
    pathPlanner->publishInitialPoseSelected(polygonsPositions[0]);
    geometry_msgs::PoseStamped initialPose;

    const Eigen::Vector3d& firstPoint = polygonsPositions[0];
    initialPose.pose.position.x = firstPoint.x();
    initialPose.pose.position.y = firstPoint.y();
    initialPose.pose.position.z = firstPoint.z();
    initialPose.pose.orientation.x = 0.0;
    initialPose.pose.orientation.y = 0.0;
    initialPose.pose.orientation.z = 0.0;
    initialPose.pose.orientation.w = 1.0;

    goal.robot_position = initialPose;
    boustrophedonServer->startPub.publish(goal.robot_position);
    boustrophedonServer->client.sendGoal(goal);
    ROS_INFO_STREAM("Sending goal");

    // Wait for the action to return
    bool finishedBeforeTimeout = boustrophedonServer->client.waitForResult(ros::Duration(5.0));
    actionlib::SimpleClientGoalState state = boustrophedonServer->client.getState();
    boustrophedon_msgs::PlanMowingPathResultConstPtr result = boustrophedonServer->client.getResult();
    if (result->plan.points.size() < 1) {
      ROS_INFO("Action did not finish before the time out.");
    } else {
      ROS_INFO("Action finished: %s", state.toString().c_str());
      cout << "Result with : " << result->plan.points.size() << endl;

      if (result->plan.points.size() > 2) {
        pathPlanner->convertStripingPlanToPath(result->plan, path);
        // pathTransformed = pathPlanner->getTransformedPath(path);
        // vector<vector<double>> vectorPathTransformed = pathPlanner->convertPathPlanToVectorVector(pathTransformed);
        // vector<double> firstQuatPos = vectorPathTransformed[0];
        // dynamicalSystem_->setPath(vectorPathTransformed);
        // boustrophedonServer->pathPub.publish(pathTransformed);
        // boustrophedonServer->closeRosLaunch();
        // checkPath = true;
      }
    }
    ros::spinOnce();
    loopRate.sleep();
  }

  cout << "path well compute" << endl;

  std::ofstream outputFile(string(WP5_TASKS_DIR) + "/txts/path.txt");
  if (!outputFile.is_open()) {
    ROS_ERROR("Unable to open file for writing.");
    return 0;
  }

  for (const auto& pose : path.poses) {
    const geometry_msgs::Point& position = pose.pose.position;
    outputFile << position.x << " " << position.y << " " << position.z << "\n";
  }

  outputFile.close();

  return 0;
}
