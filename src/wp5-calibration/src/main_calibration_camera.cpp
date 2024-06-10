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

int main(int argc, char** argv) {

  string robotName = "ur5_robot";
  double rosFreq = 300;
  int i = 0;

  // Init ros
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nh;
  ros::Rate loopRate(rosFreq);

  string path_package = string(WP5_CALIBRATION_DIR) + "/txts/path_calibration_camera.txt";

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
  std::vector<std::vector<double>> concatenatedQuatPos = pathCalibrationCamera(path_package);
  dynamicalSystem->setPath(concatenatedQuatPos);
  dynamicalSystem->setToleranceNextPoint(0.04);
  dynamicalSystem->setLinearSpeed(0.05);

  while (ros::ok() && i < concatenatedQuatPos.size()) {

    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    geometry_msgs::PoseStamped fkPose = fillPoseStamped(pairActualQuatPos.second, pairActualQuatPos.first);
    fkPose.header.stamp = ros::Time::now();
    eef_pub.publish(fkPose);

    dynamicalSystem->setCartPose(pairActualQuatPos);

    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->getLinearDsOnePosition(concatenatedQuatPos[i]);

    VectorXd twistDesiredEigen = dynamicalSystem->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJoint = roboticArm->lowLevelController(stateJoints, twistDesiredEigen);
    rosInterface->sendState(desiredJoint);

    if (dynamicalSystem->checkLinearDs() == true) {
      vector<double> stopjoint(6);
      rosInterface->sendState(stopjoint);
      cout << "point:" << i << endl;
      ros::Duration(1.5).sleep();

      std_msgs::Bool boolMsg;
      boolMsg.data = true;
      record_pub.publish(boolMsg);
      i++;
      ros::spinOnce();
      loopRate.sleep();
      cout << "boolRecordPosition:" << boolRecordPosition << endl;
      while (ros::ok() && !boolRecordPosition) {
        tuple<vector<double>, vector<double>, vector<double>> stateJoints;
        stateJoints = rosInterface->receiveState();
        vector<double> actualJoint = get<0>(stateJoints);
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
    }
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}
