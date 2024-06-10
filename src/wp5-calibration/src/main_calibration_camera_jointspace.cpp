// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <vector>

#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "RoboticArmIiwa7.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "calib.h"

using namespace std;
using namespace Eigen;

bool boolRecord = false;
bool boolRecordPosition = false;

void recordCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    boolRecord = msg->data;
  }
}
void recordPositionCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    boolRecordPosition = msg->data;
  }
}

// Function to calculate the Euclidean distance (L2 norm) error between two vectors
double calculateEuclideanError(const vector<double>& desired, const vector<double>& actual) {
  if (desired.size() != actual.size()) {
    cerr << "Error: Vectors must be of the same length." << endl;
    return -1; // Return an error code if vectors are of different lengths
  }

  double sumSquaredDifferences = 0.0;
  for (size_t i = 0; i < desired.size(); ++i) {
    double difference = desired[i] - actual[i];
    sumSquaredDifferences += difference * difference;
  }

  return sqrt(sumSquaredDifferences);
}

std::vector<std::vector<double>> readJointConfigurations(const std::string& filePath) {
  std::ifstream inFile(filePath);
  std::vector<std::vector<double>> jointPath;
  if (!inFile) {
    std::cerr << "Error opening file: " << filePath << std::endl;
    return jointPath;
  }

  std::string line;
  while (std::getline(inFile, line)) {
    std::stringstream ss(line);
    std::vector<double> jointConfig;
    double value;

    while (ss >> value) {
      jointConfig.push_back(value);
    }

    jointPath.push_back(jointConfig);
  }
  return jointPath;
}

// Function to perform linear interpolation between two joint configurations
vector<vector<double>> interpolateJointConfigurations(const vector<double>& q_start,
                                                      const vector<double>& q_end,
                                                      int n_steps) {
  vector<vector<double>> interpolated_configs(n_steps, vector<double>(q_start.size()));

  // Perform interpolation for each joint
  for (int j = 0; j < n_steps; ++j) {
    double ratio = static_cast<double>(j) / (n_steps - 1);
    for (size_t i = 0; i < q_start.size(); ++i) {
      interpolated_configs[j][i] = q_start[i] + ratio * (q_end[i] - q_start[i]);
    }
  }

  return interpolated_configs;
}

int main(int argc, char** argv) {

  string robotName = "ur5_robot";
  double rosFreq = 100;
  int i = 0;

  // Init ros
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nh;
  ros::Rate loopRate(rosFreq);

  string path_package = string(WP5_CALIBRATION_DIR) + "/txts/smoothed_trajectory.txt";

  // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("interpolated_points", 10);
  ros::Publisher eef_pub = nh.advertise<geometry_msgs::PoseStamped>("calibration/ee_pose", 10);
  ros::Publisher record_pub = nh.advertise<std_msgs::Bool>("calibration/recording", 10);
  ros::Publisher record_position_pub = nh.advertise<std_msgs::Bool>("calibration/position_recorded", 10);

  ros::Subscriber record_sub = nh.subscribe<std_msgs::Bool>("calibration/recording", 10, recordCallback);
  ros::Subscriber record_position_sub =
      nh.subscribe<std_msgs::Bool>("calibration/position_recorded", 10, recordPositionCallback);

  // set the check false
  std_msgs::Bool boolMsg;
  boolMsg.data = false;
  record_pub.publish(boolMsg);
  record_position_pub.publish(boolMsg);

  // Create an unique pointer for the instance of DynamicalSystem
  std::unique_ptr<DynamicalSystem> dynamicalSystem = nullptr;

  // Create an unique pointer for the instance of IRoboticArmBase
  std::unique_ptr<IRoboticArmBase> roboticArm = nullptr;

  // Create an unique pointer for the instance of RosInterfaceNoetic
  std::unique_ptr<RosInterfaceNoetic> rosInterface = nullptr;

  // Create an unique pointer for the instance of DynamicalSystem
  dynamicalSystem = make_unique<DynamicalSystem>(rosFreq);

  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface = make_unique<RosInterfaceNoetic>(nh, robotName);

  // Create an unique pointer for the instance of RosInterfaceNoetic
  roboticArm = make_unique<RoboticArmUr5>();

  //make the path for the around tjhe ur5 for calibration
  std::vector<std::vector<double>> jointPath = readJointConfigurations(path_package);
  std::vector<double> actualJoint(6);
  while (actualJoint[0] == 0.0 && actualJoint[1] == 0.0 && actualJoint[2] == 0.0 && actualJoint[3] == 0.0
         && actualJoint[4] == 0.0 && actualJoint[5] == 0.0) {
    ros::spinOnce();
    loopRate.sleep();
    actualJoint = get<0>(rosInterface->receiveState());
  }

  cout << "go to first pose " << endl;

  std::vector<std::vector<double>> startPath = interpolateJointConfigurations(actualJoint, jointPath[0], 500);
  i = 0;
  while (ros::ok() && i < startPath.size() - 1) {
    ros::spinOnce();
    loopRate.sleep();
    rosInterface->sendState(startPath[i]);

    if (calculateEuclideanError(startPath[i], get<0>(rosInterface->receiveState())) < 0.1) {
      i++;
    }
  }
  cout << "first pose achieved" << endl;

  i = 0;
  while (ros::ok() && i < jointPath.size()) {
    vector<double> desiredJoint = jointPath[i];
    std::vector<std::vector<double>> PathNextPoint =
        interpolateJointConfigurations(get<0>(rosInterface->receiveState()), desiredJoint, 100);

    rosInterface->sendState(PathNextPoint[0]);
    ros::spinOnce();
    int j = 0;

    while (ros::ok() && j < PathNextPoint.size() - 1) {
      ros::spinOnce();
      loopRate.sleep();
      rosInterface->sendState(PathNextPoint[j]);

      if (calculateEuclideanError(PathNextPoint[j], get<0>(rosInterface->receiveState())) < 0.1) {
        j++;
      }
    }

    cout << "point:" << i << endl;
    ros::Duration(1.5).sleep();

    std_msgs::Bool boolMsg;
    boolMsg.data = true;
    record_pub.publish(boolMsg);
    i++;
    ros::spinOnce();
    loopRate.sleep();
    while (ros::ok() && !boolRecordPosition) {
      vector<double> actualJoint = get<0>(rosInterface->receiveState());
      pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

      geometry_msgs::PoseStamped fkPose = fillPoseStamped(pairActualQuatPos.second, pairActualQuatPos.first);
      fkPose.header.stamp = ros::Time::now();
      eef_pub.publish(fkPose);

      ros::spinOnce();
      loopRate.sleep();
    }
    boolRecordPosition = false;
    boolMsg.data = false;
    record_position_pub.publish(boolMsg);
    record_pub.publish(boolMsg);
    ros::Duration(0.2).sleep();

    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}
