#include "PathPlanner.h"
//path panning functon

// Constructor definition
PathPlanner::PathPlanner(ros::NodeHandle& n, Eigen::Quaterniond target_quat, Eigen::Vector3d target_pos, std::vector<Eigen::Vector3d> polygons_positions) : 
  initialPosePub_(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10))  {

    nh=n;
    transformedPolygonPub = nh.advertise<geometry_msgs::PolygonStamped>("/flat_polygon", 1, true);
    std::string package_path = ros::package::getPath("motion_planner"); 
    // Load parameters from YAML file
    std::string yaml_path = package_path + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Access parameters from the YAML file
    limit_cycle_radius = config["limit_cycle_radius"].as<double>();
    toolOffsetFromTarget = config["toolOffsetFromTarget"].as<double>();
    flow_radius = config["flow_radius"].as<double>();
    sum_rad = flow_radius + limit_cycle_radius;

    polygonsPositions = polygons_positions;
    targetQuat = target_quat ;
    targetPos = target_pos;
    optimization_parameter();


    // FROM DS  -------------------------------------
    // pub_desired_vel_filtered = n.advertise<geometry_msgs::Pose>("/passive_control/vel_quat", 1);
    // point_pub = nh.advertise<geometry_msgs::PointStamped>("path_point", 1);
    // sub_real_pose= nh.subscribe<geometry_msgs::Pose>(robot_name + "/ee_info/Pose" , 1000, &DynamicalSystem::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());

}

// Function to find the center of the polygon
Eigen::Vector3d PathPlanner::findCenter(const std::vector<Eigen::Vector3d>& vertices) {
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    for (const auto& vertex : vertices) {
        center += vertex;
    }
    center /= static_cast<double>(vertices.size());
    return center;
}

// Function to scale a polygon around its center
void PathPlanner::scalePolygon(std::vector<Eigen::Vector3d>& vertices) {

  // Find the center of the polygon
  Eigen::Vector3d center = findCenter(vertices);
  //calculate the sacelfactor
  Eigen::Vector3d diff = center - vertices[0];
  double d = diff.norm();
  double scaleFactor = (d-optimum_radius*0.8)/d;

  
  // Translate the polygon to the origin
  for (auto& vertex : vertices) {
      vertex -= center;
  }

  // Apply scaling factor to the vertices
  for (auto& vertex : vertices) {
      vertex *= scaleFactor;
  }

  // Translate the polygon back to its original position
  for (auto& vertex : vertices) {
      vertex += center;
  }
}


std::vector<Eigen::Vector3d> PathPlanner::get_planner_points() {

    std::vector<Eigen::Vector3d> rotated_points;
    
    PathPlanner::scalePolygon(polygonsPositions);


    Eigen::Affine3d transformation = Eigen::Translation3d(targetPos(0),targetPos(1),targetPos(2)) * targetQuat.conjugate();

    Eigen::MatrixXd points_matrix(polygonsPositions.size(), 3);
    for (size_t i = 0; i < polygonsPositions.size(); ++i) {
      Eigen::Vector3d rotated_point = transformation.inverse() * polygonsPositions[i];
      rotated_points.push_back(rotated_point);
    }
    return rotated_points;
}

boustrophedon_msgs::PlanMowingPathGoal  PathPlanner::ComputeGoal() {
  flatPolygons = get_planner_points();

  Eigen::Vector3d p1 = flatPolygons[0];
  Eigen::Vector3d p2 = flatPolygons[1];
  Eigen::Vector3d p3 = flatPolygons[2];
  Eigen::Vector3d p4 = flatPolygons[3];
  //polygon for the server
  boustrophedon_msgs::PlanMowingPathGoal goal;

  goal.property.header.stamp = ros::Time::now();
  goal.property.header.frame_id = "base";
  goal.property.polygon.points.resize(4);
  goal.property.polygon.points[0].x = p1(0);
  goal.property.polygon.points[0].y = p1(1);
  goal.property.polygon.points[0].z = p1(2);
  goal.property.polygon.points[1].x = p2(0);
  goal.property.polygon.points[1].y = p2(1);
  goal.property.polygon.points[1].z = p2(2);           
  goal.property.polygon.points[2].x = p3(0);
  goal.property.polygon.points[2].y = p3(1);
  goal.property.polygon.points[2].z = p3(2);
  goal.property.polygon.points[3].x = p4(0);
  goal.property.polygon.points[3].y = p4(1);
  goal.property.polygon.points[3].z = p4(2);

  goal.robot_position.pose.orientation.x = 0.0;
  goal.robot_position.pose.orientation.y = 0.0;
  goal.robot_position.pose.orientation.z = 0.0;
  goal.robot_position.pose.orientation.w = 1.0;

  return goal;
}

double PathPlanner::find_height() {
    if (polygonsPositions.empty()) {
        // Handle the case when there are no points
        return 0.0;
    }

    double maxZ = -std::numeric_limits<double>::infinity();
    double minZ = std::numeric_limits<double>::infinity();
    std::vector<Eigen::Vector3d> lowestZPoints;
    std::vector<Eigen::Vector3d> highestZPoints;

    for (const auto& point : polygonsPositions) {
        if (point.z() > maxZ) {
            maxZ = point.z();
            highestZPoints.clear();
            highestZPoints.push_back(point);
        } else if (point.z() < minZ) {
            minZ = point.z();
            lowestZPoints.clear();
            lowestZPoints.push_back(point);
        }
    }

    if (lowestZPoints.empty() || highestZPoints.empty()) {
        // Handle the case when vectors are empty (no points found)
        return 0.0;
    }

    Eigen::Vector3d diff = lowestZPoints[0] - highestZPoints[0];
    double height = diff.norm();
    return height;
}


int PathPlanner::optimization_parameter() {
  double D = find_height();

  if (D == 0.0){
    optimum_radius =  sum_rad;
    return 0;
  }
  double d = sum_rad;
  double n = D / d;
  int roundedNumber = std::round(n)+4 ;
  double r_new = D / roundedNumber ;
  optimum_radius = r_new;
  return 1;

}


void PathPlanner::publishInitialPose() {
  double maxZ = -std::numeric_limits<double>::infinity();
  std::vector<Eigen::Vector3d> highestZPoints;
  int imax = 0;
  int i = 0;

  std::vector<Eigen::Vector3d> points = polygonsPositions;
  for (const auto& point : points) {
    if (point.z() > maxZ) {
      maxZ = point.z();
      highestZPoints.clear();  // Clear previous points with lower z
      highestZPoints.push_back(point);
      imax = i;
    } 
    i++;
  }
  std::vector<Eigen::Vector3d>  flatPolygons = get_planner_points();
  Eigen::Vector3d pointInitial = flatPolygons[imax];
  see_target_flat();
  std::cout<<points[imax]<< std::endl;
  double delta = 0.1;
  // Create a publisher for the /initialpose topic

  // Create and fill the message
  initialPoseMsg.header.seq = 0;
  initialPoseMsg.header.stamp = ros::Time(0);
  initialPoseMsg.header.frame_id = "base";

  initialPoseMsg.pose.pose.position.x = pointInitial(0)+ delta;
  initialPoseMsg.pose.pose.position.y = pointInitial(1)- delta;
  initialPoseMsg.pose.pose.position.z = pointInitial(2);

  initialPoseMsg.pose.pose.orientation.x = 0.0;
  initialPoseMsg.pose.pose.orientation.y = 0.0;
  initialPoseMsg.pose.pose.orientation.z = 0.0;
  initialPoseMsg.pose.pose.orientation.w = 1.0;

  initialPoseMsg.pose.covariance.fill(0.0);  // Fill the covariance with zeros

  initialPosePub_.publish(initialPoseMsg);
  initialPose.header = initialPoseMsg.header;
  initialPose.pose = initialPoseMsg.pose.pose;
}

nav_msgs::Path PathPlanner::get_transformed_path(const nav_msgs::Path& originalPath) {
    nav_msgs::Path transformedPath;
    Eigen::Affine3d transformation = Eigen::Translation3d(targetPos(0),targetPos(1),targetPos(2)) * targetQuat.conjugate();;

    transformedPath.header = originalPath.header;

    for (const auto& pose : originalPath.poses) {
      // Convert pose to Eigen types for transformation
      Eigen::Vector3d originalPosition(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      Eigen::Quaterniond originalOrientation(pose.pose.orientation.w, pose.pose.orientation.x,
                                              pose.pose.orientation.y, pose.pose.orientation.z);

      // Apply transformation
      Eigen::Vector3d transformedPosition = transformation * originalPosition;
      // std::cout << originalOrientation.w() << std::endl;
      
      // Convert the rotation matrix to a quaternion before applying the rotation
      // Eigen::Quaterniond transformedOrientation(transformation.rotation());
      // transformedOrientation = transformedOrientation * originalOrientation;
      Eigen::Quaterniond transformedOrientation = targetQuat;

      // Convert back to geometry_msgs types
      geometry_msgs::PoseStamped transformedPose;
      transformedPose.header = originalPath.header;
      transformedPose.pose.position.x = transformedPosition.x();
      transformedPose.pose.position.y = transformedPosition.y();
      transformedPose.pose.position.z = transformedPosition.z();
      transformedPose.pose.orientation.w = transformedOrientation.w();
      transformedPose.pose.orientation.x = transformedOrientation.x();
      transformedPose.pose.orientation.y = transformedOrientation.y();
      transformedPose.pose.orientation.z = transformedOrientation.z();

      // Add the transformed pose to the new path
      transformedPath.poses.push_back(transformedPose);
    }
    geometry_msgs::PoseStamped first_pose = transformedPath.poses[0];

    // Extract position (x, y, z)
    double x = first_pose.pose.position.x;
    double y = first_pose.pose.position.y;
    double z = first_pose.pose.position.z;
    firstPos = {x,y,z};
    // set_strategique_position();
    return transformedPath;
}

void PathPlanner::see_target_flat(){
    geometry_msgs::PolygonStamped visualpolygonTarget;
    visualpolygonTarget.header.frame_id = "base";  
    visualpolygonTarget.header.stamp = ros::Time::now();

    for (const auto& point : flatPolygons) {
        geometry_msgs::Point32 msg_point;
        msg_point.x = point.x();
        msg_point.y = point.y();
        msg_point.z = point.z();
        visualpolygonTarget.polygon.points.push_back(msg_point);
    }
    transformedPolygonPub.publish(visualpolygonTarget);
}

// void PathPlanner::set_strategique_position(){
//     // Set values for a initial orientation
//     std::vector<double> parameter_quat = {targetQuat.x(),targetQuat.y(),targetQuat.z(),targetQuat.w()};
//     nh.setParam("/initialQuat", parameter_quat);

//     Eigen::Matrix3d rotationMatrix = targetQuat.toRotationMatrix();
//     Eigen::Vector3d firstPosEigen(firstPos[0],firstPos[1],firstPos[2]);

//     Eigen::Vector3d parameter_pos3f = firstPosEigen - toolOffsetFromTarget  *  rotationMatrix.col(2);
    
//     std::vector<double> parameter_pos;
//     parameter_pos.reserve(parameter_pos3f.size());  // Reserve space for efficiency
//     for (int i = 0; i < parameter_pos3f.size(); ++i) {
//         parameter_pos.push_back(static_cast<double>(parameter_pos3f[i]));
//     }
//     nh.setParam("/initialPos", parameter_pos);
//     nh.setParam("/finalPos", parameter_pos);
// }
 // server has a service to convert StripingPlan to Path, but all it does it call this method
bool PathPlanner::convertStripingPlanToPath(const boustrophedon_msgs::StripingPlan& striping_plan, nav_msgs::Path& path)
{
  path.header.frame_id = striping_plan.header.frame_id;
  path.header.stamp = striping_plan.header.stamp;

  path.poses.clear();
  for (std::size_t i = 0; i < striping_plan.points.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = striping_plan.header.frame_id;
    pose.header.stamp = striping_plan.header.stamp;
    pose.pose.position = striping_plan.points[i].point;

    if (i < striping_plan.points.size() - 1)
    {
      double dx = striping_plan.points[i + 1].point.x - striping_plan.points[i].point.x;
      double dy = striping_plan.points[i + 1].point.y - striping_plan.points[i].point.y;
      double dz = striping_plan.points[i + 1].point.z - striping_plan.points[i].point.z;

      pose.pose.orientation = headingToQuaternion(dx, dy, dz);
    }
    else
    {
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
    }

    path.poses.push_back(pose);
  }

  return true;
}
bool PathPlanner::convertStripingPlanToVectorVector(const boustrophedon_msgs::StripingPlan& striping_plan)
{
  size_t size = striping_plan.points.size();
  std::vector<std::vector<double>> path(size);
  std::vector<double> quatPos ;

  for (std::size_t i = 0; i < size; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = striping_plan.header.frame_id;
    pose.header.stamp = striping_plan.header.stamp;
    pose.pose.position = striping_plan.points[i].point;

    if (i < striping_plan.points.size() - 1)
    {
      double dx = striping_plan.points[i + 1].point.x - striping_plan.points[i].point.x;
      double dy = striping_plan.points[i + 1].point.y - striping_plan.points[i].point.y;
      double dz = striping_plan.points[i + 1].point.z - striping_plan.points[i].point.z;

      pose.pose.orientation = headingToQuaternion(dx, dy, dz);
      quatPos.push_back(pose.pose.orientation.x);
      quatPos.push_back(pose.pose.orientation.y);
      quatPos.push_back(pose.pose.orientation.z);
      quatPos.push_back(pose.pose.orientation.w);
    }
    else
    {
      quatPos.push_back(0.0);
      quatPos.push_back(0.0);
      quatPos.push_back(0.0);
      quatPos.push_back(1.0);
    }
    quatPos.push_back(pose.pose.position.x);
    quatPos.push_back(pose.pose.position.y);
    quatPos.push_back(pose.pose.position.z);

    path.push_back(quatPos);
  }

  return true;
}

geometry_msgs::Quaternion PathPlanner::headingToQuaternion(double x, double y, double z)
{
  // get orientation from heading vector
  const tf2::Vector3 heading_vector(x, y, z);
  const tf2::Vector3 origin(1, 0, 0);

  const auto w = (origin.length() * heading_vector.length()) + tf2::tf2Dot(origin, heading_vector);
  const tf2::Vector3 a = tf2::tf2Cross(origin, heading_vector);
  tf2::Quaternion q(a.x(), a.y(), a.z(), w);
  q.normalize();

  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w()))
  {
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    q.setW(1);
  }

  return tf2::toMsg(q);
}
