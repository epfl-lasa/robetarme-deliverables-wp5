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

  double deltaTime = 0.1;

  // init ros
  ros::init(argc, argv, "main_temporary");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1 / deltaTime);

  // 1) init class for shotcrete -----------------------------------------

  // Create an unique pointer for the instance of IRoboticArmBase
  std::unique_ptr<IRoboticArmBase> roboticArm = nullptr;
  roboticArm = make_unique<RoboticArmUr5>();
  // Create an unique pointer for the instance of RosInterfaceNoetic
  std::unique_ptr<RosInterfaceNoetic> rosInterface = nullptr;
  rosInterface = make_unique<RosInterfaceNoetic>(nh);
  // Create an unique pointer for the instance of DynamicalSystem
  std::unique_ptr<DynamicalSystem> dynamicalSystem = nullptr;
  dynamicalSystem = make_unique<DynamicalSystem>(1 / deltaTime);
  // Create an unique pointer for the instance of TargetExtraction
  std::unique_ptr<TargetExtraction> targetextraction = nullptr;
  targetextraction = make_unique<TargetExtraction>(nh);
  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<PathPlanner> pathplanner = nullptr;
  pathplanner = make_unique<PathPlanner>(nh);
  // Create an unique pointer for the instance of PathPlanner
  std::unique_ptr<BoustrophedonServer> boustrophedonserver = nullptr;
  boustrophedonserver = make_unique<BoustrophedonServer>(nh);

  // 2) comput path -----------------------------------------

  // extract polygons for boustrophedon
  // WARNING: need the position of the target from Optitrack to continue
  std::vector<Eigen::Vector3d> polygons_positions = targetextraction->get_polygons();
  Eigen::Quaterniond quatTarget = targetextraction->get_quat_target();
  Eigen::Vector3d posTarget = targetextraction->get_pos_target();
  targetextraction->see_target();

  // initialization
  pathplanner->setTarget(quatTarget, posTarget, polygons_positions);
  double optimum_radius = pathplanner->getOptimumRadius();

  boustrophedonserver->setOptimumRad(optimum_radius);
  // wait for the action server to startnew_rad

  std::cout << "Waiting for action server to start." << std::endl;
  boustrophedonserver->initRosLaunch();

  std::cout << "Action server started" << std::endl;

  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal = pathplanner->ComputeGoal();
  boustrophedonserver->polygon_pub.publish(goal.property);

  std::cout << "Waiting for goal" << std::endl;
 
  nav_msgs::Path path;
  nav_msgs::Path path_transformed;

  while (ros::ok()) {
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

      std::cout << "Result with : " << result->plan.points.size() << std::endl;

      if (result->plan.points.size() > 2) {
        pathplanner->convertStripingPlanToPath(result->plan, path);
        path_transformed = pathplanner->get_transformed_path(path);
        std::vector<std::vector<double>> vectorPathTransformed = pathplanner->convertPathPlanToVectorVector(path_transformed);

        dynamicalSystem->set_path(vectorPathTransformed);

        boustrophedonserver->path_pub.publish(path_transformed);

        boustrophedonserver->closeRosLaunch();
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  // nav_msgs::Path path;
  // nav_msgs::Path path_transformed;
  // n.setParam("/startDS", false);
  // n.setParam("/finishDS", false);

  // BoustrophedonServer boustrophedonServer(nh, 0.5);

  while (ros::ok()) {
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();

    vector<double>& retrievedPosition = get<0>(stateJoints);
    vector<double>& retrievedSpeed = get<1>(stateJoints);
    vector<double>& retrievedTorque = get<2>(stateJoints);
    std::cout << "retrievedPosition:" << retrievedPosition[3] << std::endl;
    vector<double> posCart = roboticArm->getFK(retrievedPosition);
    std::cout << "posCart:" << posCart[5] << std::endl;

    ros::spinOnce();// Allow the message to be subscribed
    loop_rate.sleep();
  }

  return 0;
}
