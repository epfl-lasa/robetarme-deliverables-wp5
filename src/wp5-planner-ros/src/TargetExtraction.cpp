#include "TargetExtraction.h"

//TargetExrtaction functon
TargetExtraction::TargetExtraction(ros::NodeHandle& nh)
    : poseTargetSub(nh.subscribe("/vrpn_client_node/TargetRobetarme/pose_transform", 10, &TargetExtraction::CC_vrpn_target, this)){

    originalPolygonPub = nh.advertise<geometry_msgs::PolygonStamped>("/original_polygon", 1, true);
    // Get the path to the package
    std::string package_path = ros::package::getPath("wp5-planner-ros"); // Replace "your_package" with your actual package name

    // Load parameters from YAML file
    std::string yaml_path = package_path + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Access parameters from the YAML file
    height_target = config["height_target"].as<double>();
    width_target = config["width_target"].as<double>();

    std::cout << "Waiting for target Pose" << std::endl;

    while(!targetReceived || !ros::ok()){
      ros::spinOnce();
    }
    std::cout << "rostopic for the target received" << std::endl;
}

std::vector<Eigen::Vector3d> TargetExtraction::get_polygons() {
    // Desired displacements
    std::vector<Eigen::Vector3d> displacements{
        Eigen::Vector3d(width_target / 2.0, height_target / 2.0 , 0),
        Eigen::Vector3d(-width_target / 2.0, height_target / 2.0, 0),
        Eigen::Vector3d(-width_target / 2.0, -height_target / 2.0, 0),
        Eigen::Vector3d(width_target / 2.0, -height_target / 2.0, 0)
    };

    // Extract position and ensure quaternion is normalized
    Eigen::Vector3d position = targetPos;
    Eigen::Quaterniond normalizedQuat = targetQuat.normalized();
    Eigen::Matrix3d rotation_matrix = normalizedQuat.toRotationMatrix();

    // Calculate new positions
    polygons_positions.clear();  // Clear existing positions
    for (const auto& displacement : displacements) {
        polygons_positions.push_back(position + rotation_matrix * displacement);
    }
    std::cout<<"Polygons well computed"<<std::endl;
    see_target();
    return polygons_positions;
}

Eigen::Quaterniond TargetExtraction::get_quat_target() {
    return targetQuat;
}

Eigen::Vector3d TargetExtraction::get_pos_target() {
    return targetPos;
}

void TargetExtraction::CC_vrpn_target(const geometry_msgs::PoseStamped::ConstPtr msg) {
    targetPos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    targetQuat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    targetReceived = true;
}

void TargetExtraction::see_target(){

      geometry_msgs::PolygonStamped visualpolygonTarget;
      visualpolygonTarget.header.frame_id = "base";  
      visualpolygonTarget.header.stamp = ros::Time::now();

      for (const auto& point : polygons_positions) {
          geometry_msgs::Point32 msg_point;
          msg_point.x = point.x();
          msg_point.y = point.y();
          msg_point.z = point.z();
          visualpolygonTarget.polygon.points.push_back(msg_point);
      }
      originalPolygonPub.publish(visualpolygonTarget);
  }
