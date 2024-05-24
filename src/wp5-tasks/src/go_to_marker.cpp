// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <tuple>

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

class PoseSubscriber {
public:
  vector<double> desiredQuatPos;
  PoseSubscriber() : nh_("~") {
    // Subscribe to the pose topic
    sub_ = nh_.subscribe("pose_topic", 1000, &PoseSubscriber::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    desiredQuatPos = [
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w,
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z
    ];
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv) {
  double rosFreq = 300;
  int i = 0;
  string robotName = "ur5_robot";

  // Init ros
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nh;
  ros::Rate loopRate(rosFreq);

  PoseSubscriber pose_subscriber;
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
  dynamicalSystem->setToleranceNextPoint(0.05);
  dynamicalSystem->setLinearSpeed(0.05);

  while (ros::ok()) {

    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receiveState();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed =
        dynamicalSystem->getLinearDsOnePosition(pose_subscriber.desiredQuatPos);

    VectorXd twistDesiredEigen = dynamicalSystem->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);

    vector<double> desiredJoint = roboticArm->lowLevelController(stateJoints, twistDesiredEigen);
    rosInterface->sendState(desiredJoint);
  }
  return 0;
}