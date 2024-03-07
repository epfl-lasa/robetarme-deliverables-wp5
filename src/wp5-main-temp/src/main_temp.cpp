// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include "BoustrophedonServer.h"
#include "DynamicalSystem.h"
#include "IRoboticArmBase.h"
#include "PathPlanner.h"
#include "RoboticArmUr5.h"
#include "RosInterfaceNoetic.h"
#include "TargetExtraction.h"
#include "ros/ros.h"
#include <tuple>

int main(int argc, char** argv) {
  ros::init(argc, argv, "main_temporary");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  // Create an instance of RosInterfaceNoetic
  RosInterfaceNoetic rosInterface(nh);
  IRoboticArmBase iRoboticArmBase;
  // RoboticArmUr5 ur5Arm;

  // TargetExtraction targetextraction(nh);

  tuple<vector<double>, vector<double>, vector<double>> stateJoints;
  while (ros::ok()) {
    stateJoints = rosInterface.receive_state();

    vector<double>& retrievedPosition = get<0>(stateJoints);
    vector<double>& retrievedSpeed = get<1>(stateJoints);
    vector<double>& retrievedTorque = get<2>(stateJoints);
    std::cout << "retrievedPosition:" << retrievedPosition[3] << std::endl;
    // vector<double> posCart = ur5Arm.getFK(retrievedPosition);
    // std::cout << "posCart:"<<posCart[3]<< std::endl;

    ros::spinOnce();// Allow the message to be subscribed
    loop_rate.sleep();
  }

  return 0;
}
