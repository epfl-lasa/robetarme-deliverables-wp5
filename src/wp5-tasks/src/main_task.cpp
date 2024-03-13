// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include "Tasks.h"
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
  double rosFreq= 1/deltaTime;
  // init ros
  ros::init(argc, argv, "main_tasks");
  ros::NodeHandle nh;

  ros::Rate loop_rate(1 / deltaTime);


// CHECK RVIZ -------------------------------------
  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("path_point", 1);
  ros::Publisher pub_desired_vel_filtered = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100 );
//------------------------------

  //init class for Tasks -----------------------------------------
  unique_ptr<Tasks> tasks = nullptr;
  tasks = make_unique<Tasks>(nh, rosFreq);
  
  // comput path -----------------------------------------
  tasks->computePathShotcrete();

  //init shotcrete
  bool valid = tasks->initShotcrete();

  if  (valid){
    cout  <<"Iniitalization shotcrete  ok"<< endl;
  }
  else{
    cout  <<"Iniitalization shotcrete  failed"<< endl;
    return  0;
  }

  // go first position  
  valid = tasks->goFirstPosition();
  if  (valid){
    cout  <<"We are in the first position"<< endl;
  }
  else{
    cout  <<"failed to go into first position"<< endl;
    return  0;
  }

  // Do shotcrete  
  valid = tasks->DoShotcrete();
  if  (valid){
    cout  <<"shotcrete Done"<< endl;
  }
  else{
    cout  <<"failed to perform shotcrete, go to home"<< endl;
    valid = tasks->goHome();

    return  0;
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
