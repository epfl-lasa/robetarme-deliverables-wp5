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
#include <ros/ros.h>
#include <tuple>


#include <geometry_msgs/PointStamped.h> //<----------to remove
#include <geometry_msgs/Point.h> //<----------to remove
#include "visualization_msgs/Marker.h"


using namespace std;
using namespace Eigen;

void publishPointStamped(const Eigen::Vector3d&  pathPoint,  ros::Publisher pointPub);
void twistMarker(VectorXd twistDesiredEigen, vector<double> pos, ros::Publisher& marker_pub);


int main(int argc, char** argv) {

  double deltaTime = 0.01;

  // init ros
  ros::init(argc, argv, "main_temporary");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1 / deltaTime);


// CHECK RVIZ -------------------------------------
  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("path_point", 1);
  ros::Publisher pub_desired_vel_filtered = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100 );


//------------------------------

  // 1) init class for shotcrete -----------------------------------------

  // Create an unique pointer for the instance of IRoboticArmBase
  unique_ptr<IRoboticArmBase> roboticArm = nullptr;
  roboticArm = make_unique<RoboticArmUr5>();
  // Create an unique pointer for the instance of RosInterfaceNoetic
  unique_ptr<RosInterfaceNoetic> rosInterface = nullptr;
  rosInterface = make_unique<RosInterfaceNoetic>(nh);
  // Create an unique pointer for the instance of DynamicalSystem
  unique_ptr<DynamicalSystem> dynamicalSystem = nullptr;
  dynamicalSystem = make_unique<DynamicalSystem>(1 / deltaTime);
  // Create an unique pointer for the instance of TargetExtraction
  unique_ptr<TargetExtraction> targetextraction = nullptr;
  targetextraction = make_unique<TargetExtraction>(nh);
  // Create an unique pointer for the instance of PathPlanner
  unique_ptr<PathPlanner> pathplanner = nullptr;
  pathplanner = make_unique<PathPlanner>(nh);
  // Create an unique pointer for the instance of PathPlanner
  unique_ptr<BoustrophedonServer> boustrophedonserver = nullptr;
  boustrophedonserver = make_unique<BoustrophedonServer>(nh);

  // 2) comput path -----------------------------------------

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

      cout << "Result with : " << result->plan.points.size() << endl;

      if (result->plan.points.size() > 2) {
        pathplanner->convertStripingPlanToPath(result->plan, path);

        path_transformed = pathplanner->get_transformed_path(path);

        vector<vector<double>> vectorPathTransformed = pathplanner->convertPathPlanToVectorVector(path_transformed);

        vector<double> firstQuatPos = vectorPathTransformed[0];

        dynamicalSystem->set_path(vectorPathTransformed);

        boustrophedonserver->path_pub.publish(path_transformed);

        boustrophedonserver->closeRosLaunch();
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  // 3) shotcreete -------------------------------------------------


  while (ros::ok()) {
    // set and get desired speed
    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    stateJoints = rosInterface->receive_state();
    vector<double> actualJoint = get<0>(stateJoints);
    pair<Quaterniond, Vector3d> pairActualQuatPos = roboticArm->getFK(actualJoint);

    dynamicalSystem->setCartPose(pairActualQuatPos);
    pair<Quaterniond, Vector3d> pairQuatLinerSpeed = dynamicalSystem->get_DS_quat_speed();


    VectorXd twistDesiredEigen = roboticArm->getTwistFromDS(pairActualQuatPos.first, pairQuatLinerSpeed);
    vector<double> desiredJointSpeed = roboticArm->getIDynamics(actualJoint, twistDesiredEigen);

    // rosInterface->send_state(desiredJointSpeed);

    //ros checkup
    publishPointStamped(dynamicalSystem->pathPoint,  point_pub);
    Vector3d poseigen =pairActualQuatPos.second;
    vector<double> pos = {poseigen(0),poseigen(1),poseigen(2)};
    twistMarker(twistDesiredEigen, pos, pub_desired_vel_filtered);


    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}



void publishPointStamped(const Vector3d&  pathPoint,  ros::Publisher pointPub) {

geometry_msgs::PointStamped point_stamped_msg;
point_stamped_msg.header.stamp = ros::Time::now();
point_stamped_msg.header.frame_id = "base"; // Set your desired frame_id

// Assign Eigen vector components to PointStamped message
point_stamped_msg.point.x = pathPoint(0);
point_stamped_msg.point.y = pathPoint(1);
point_stamped_msg.point.z = pathPoint(2);

// Publish the PointStamped message
pointPub.publish(point_stamped_msg);
}


void twistMarker(VectorXd twistDesiredEigen,vector<double> pos, ros::Publisher& marker_pub) {
    visualization_msgs::Marker linear_marker, angular_marker;


    // Linear twist arrow marker
    linear_marker.header.frame_id = "base"; // Set your desired frame ID
    linear_marker.header.stamp = ros::Time();
    linear_marker.ns = "twist";
    linear_marker.id = 0;
    linear_marker.type = visualization_msgs::Marker::ARROW;
    linear_marker.action = visualization_msgs::Marker::ADD;
    linear_marker.color.r = 1.0;
    linear_marker.color.g = 1.0;
    linear_marker.color.b = 0.0;
    linear_marker.color.a = 1.0; // Don't forget to set the alpha!

    linear_marker.scale.x = 0.05; // Arrow width
    linear_marker.scale.y = 0.01; // Arrow head width
    linear_marker.scale.z = 0.5; // Arrow head length



    linear_marker.pose.orientation.w = 1.0;
    linear_marker.pose.position.x = pos[0];
    linear_marker.pose.position.y = pos[1];
    linear_marker.pose.position.z = pos[2];

    linear_marker.points.push_back(geometry_msgs::Point());
    geometry_msgs::Point point;
    point.x = twistDesiredEigen(3);
    point.y = twistDesiredEigen(4);
    point.z =  twistDesiredEigen(5);
    linear_marker.points.push_back(point);

    // Angular twist arrow marker
    angular_marker = linear_marker; // Copy settings from linear_marker
    angular_marker.id = 1;
    angular_marker.color.r = 1.0;
    angular_marker.color.g = 0.0;
    angular_marker.color.b = 0.0;

    // Angular twist direction
    angular_marker.points.push_back(geometry_msgs::Point());
    point.x = twistDesiredEigen(0);
    point.y = twistDesiredEigen(1);
    point.z =  twistDesiredEigen(2);
    angular_marker.points.push_back(point);

    // Publish markers
    marker_pub.publish(linear_marker);
    //marker_pub.publish(angular_marker);
}
