#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

std::vector<std::vector<double>> interpolatePoints(const Eigen::Vector3d& p0,
                                                   const Eigen::Vector3d& p1,
                                                   const Eigen::Vector3d& p2,
                                                   const Eigen::Vector3d& p3,
                                                   int numSteps) {
  std::vector<std::vector<double>> interpolatedPoints;
  for (int i = 0; i <= numSteps; ++i) {
    double t = static_cast<double>(i) / numSteps;
    Eigen::Vector3d interpolated_point = p0 * (1.0 - t) * (1.0 - t) * (1.0 - t) + 3 * p1 * (1.0 - t) * (1.0 - t) * t
                                         + 3 * p2 * (1.0 - t) * t * t + p3 * t * t * t;
    std::vector<double> point = {interpolated_point.x(), interpolated_point.y(), interpolated_point.z()};
    interpolatedPoints.push_back(point);
  }
  return interpolatedPoints;
}

std::vector<std::vector<double>> interpolate2Points(const Eigen::Vector3d& p0,
                                                    const Eigen::Vector3d& p1,
                                                    int numSteps) {
  std::vector<std::vector<double>> interpolatedPoints;
  for (int i = 0; i <= numSteps; ++i) {
    double t = static_cast<double>(i) / numSteps;
    double t2 = t * t;
    double t3 = t2 * t;
    Eigen::Vector3d interpolated_point =
        (2 * t3 - 3 * t2 + 1) * p0 + (t3 - 2 * t2 + t) * (p1 - p0) + (-2 * t3 + 3 * t2) * p1 + (t3 - t2) * (p0 - p1);
    std::vector<double> point = {interpolated_point.x(), interpolated_point.y(), interpolated_point.z()};
    interpolatedPoints.push_back(point);
  }
  return interpolatedPoints;
}

std::vector<std::vector<double>> interpolateQuaternions(const Eigen::Quaterniond& quaternion1,
                                                        const Eigen::Quaterniond& quaternion2,
                                                        int numSteps) {

  std::vector<std::vector<double>> interpolatedQuaternions;
  for (int i = 0; i <= numSteps; ++i) {
    double t = static_cast<double>(i) / numSteps;
    Eigen::Quaterniond interpolated_quaternion = quaternion1.slerp(t, quaternion2);
    std::vector<double> quaternion = {interpolated_quaternion.x(),
                                      interpolated_quaternion.y(),
                                      interpolated_quaternion.z(),
                                      interpolated_quaternion.w()};
    interpolatedQuaternions.push_back(quaternion);
  }
  return interpolatedQuaternions;
}

vector<vector<double>> pathCalibration() {
  double size = 0.8;
  double heigtStep = 0.2;
  double heigtminimal = 0.5;
  double i = 1;
  int n = 15;
  std::vector<std::vector<double>> concatenatedQuatPos;

  Eigen::Vector3d pNull(0, 0, 0);

  // Define 4 points
  Eigen::Vector3d p0(0, size, heigtminimal);
  Eigen::Vector3d p1(size, size, heigtminimal);
  Eigen::Vector3d p2(size, -size, heigtminimal);
  Eigen::Vector3d p3(0, -size, heigtminimal);

  Eigen::Quaterniond q1(-0.5, 0.5, 0.5, 0.5);
  Eigen::Quaterniond q2(0.5, 0.5, -0.5, 0.5);
  Eigen::Quaterniond q3(0.0, 0.0, 0.7, -0.7);
  Eigen::Quaterniond q4(0.7, -0.7, 0.0, 0.0);
  Eigen::Quaterniond q5(0.9, -0.4, 0.1, 0.2);
  Eigen::Quaterniond q6(-0.2, 0.0, -0.5, 0.8);

  // Interpolate points
  std::vector<std::vector<double>> interpolatedPoints1 = interpolatePoints(p0, p1, p2, p3, n);
  std::vector<std::vector<double>> interpolateQuaternions1 = interpolateQuaternions(q1, q2, n);

  // Concatenate interpolated quaternions and points
  for (size_t i = 0; i < interpolatedPoints1.size(); ++i) {
    std::vector<double> data;
    data.insert(data.end(), interpolateQuaternions1[i].begin(), interpolateQuaternions1[i].end());
    data.insert(data.end(), interpolatedPoints1[i].begin(), interpolatedPoints1[i].end());
    concatenatedQuatPos.push_back(data);
  }

  // Define 4 points
  Eigen::Vector3d p31(0, size, heigtminimal + i * heigtStep);
  Eigen::Vector3d p21(size, size, heigtminimal + i * heigtStep);
  Eigen::Vector3d p11(size, -size, heigtminimal + i * heigtStep);
  Eigen::Vector3d p01(0, -size, heigtminimal + i * heigtStep);

  i++;
  std::vector<std::vector<double>> interpolatedPoints2 = interpolatePoints(p01, p11, p21, p31, n);
  std::vector<std::vector<double>> interpolateQuaternions2 = interpolateQuaternions(q3, q4, n);

  // Concatenate interpolated quaternions and points
  for (size_t i = 0; i < interpolatedPoints2.size(); ++i) {
    std::vector<double> data;
    data.insert(data.end(), interpolateQuaternions2[i].begin(), interpolateQuaternions2[i].end());
    data.insert(data.end(), interpolatedPoints2[i].begin(), interpolatedPoints2[i].end());
    concatenatedQuatPos.push_back(data);
  }
  size *= 0.8;
  Eigen::Vector3d p02(0, size, heigtminimal + i * heigtStep);
  Eigen::Vector3d p12(size, size, heigtminimal + i * heigtStep);
  Eigen::Vector3d p22(size, -size, heigtminimal + i * heigtStep);
  Eigen::Vector3d p32(0, -size, heigtminimal + i * heigtStep);

  std::vector<std::vector<double>> interpolatedPoints3 = interpolatePoints(p02, p12, p22, p32, n);
  std::vector<std::vector<double>> interpolateQuaternions3 = interpolateQuaternions(q5, q6, n);

  // Concatenate interpolated quaternions and points
  for (size_t i = 0; i < interpolatedPoints3.size(); ++i) {
    std::vector<double> data;
    data.insert(data.end(), interpolateQuaternions3[i].begin(), interpolateQuaternions3[i].end());
    data.insert(data.end(), interpolatedPoints3[i].begin(), interpolatedPoints3[i].end());
    concatenatedQuatPos.push_back(data);
  }

  return concatenatedQuatPos;
}

vector<vector<double>> pathCalibrationCamera() {
  double size = 0.8;
  double heigtStep = 0.2;
  double heigtminimal = 0.5;
  double i = 1;
  int n = 12;
  std::vector<std::vector<double>> concatenatedQuatPos;

  Vector3d p1(0.27324454341372706, 0.19464807091924, 1.0661243466183243);
  Quaterniond q1(0.19370993031229597, 0.74541521182828, -0.2118323970790053, -0.5564109821034878);
  Vector3d p2(0.2732354992385943, 0.19453366119625992, 0.7480607749393439);
  Quaterniond q2(-0.19414157222403933, 0.7450078313135486, 0.3123063452536237, -0.5565403201298049);
  Vector3d p3(0.48459948593552965, 0.4114042526763986, 0.7938182587056525);
  Quaterniond q3(-0.1916888094272235, 0.7468143233892723, 0.3102938215488192, -0.5560948759246273);
  Vector3d p4(0.6629256296089461, 0.05162022661732567, 0.9853390823848523);
  Quaterniond q4(-0.1913603335637698, 0.7466248662326888, 0.31030201629895104, -0.5564577167603915);
  Vector3d p5(0.3522821119794277, 0.24078264008416814, 1.077835586117339);
  Quaterniond q5(0.6, 0.4239406184741184, -0.5109122142734902, -0.5593981937893171);

  // Interpolate points
  std::vector<std::vector<double>> interpolatedPoints1 = interpolate2Points(p1, p2, n);
  std::vector<std::vector<double>> interpolateQuaternions1 = interpolateQuaternions(q1, q2, n);

  // Concatenate interpolated quaternions and points
  for (size_t i = 0; i < interpolatedPoints1.size(); ++i) {
    std::vector<double> data;
    data.insert(data.end(), interpolateQuaternions1[i].begin(), interpolateQuaternions1[i].end());
    data.insert(data.end(), interpolatedPoints1[i].begin(), interpolatedPoints1[i].end());
    concatenatedQuatPos.push_back(data);
  }

  std::vector<std::vector<double>> interpolatedPoints2 = interpolate2Points(p2, p3, n);
  std::vector<std::vector<double>> interpolateQuaternions2 = interpolateQuaternions(q2, q3, n);

  // Concatenate interpolated quaternions and points
  for (size_t i = 0; i < interpolatedPoints2.size(); ++i) {
    std::vector<double> data;
    data.insert(data.end(), interpolateQuaternions2[i].begin(), interpolateQuaternions2[i].end());
    data.insert(data.end(), interpolatedPoints2[i].begin(), interpolatedPoints2[i].end());
    concatenatedQuatPos.push_back(data);
  }

  std::vector<std::vector<double>> interpolatedPoints3 = interpolate2Points(p3, p4, n);
  std::vector<std::vector<double>> interpolateQuaternions3 = interpolateQuaternions(q3, q4, n);

  // Concatenate interpolated quaternions and points
  for (size_t i = 0; i < interpolatedPoints3.size(); ++i) {
    std::vector<double> data;
    data.insert(data.end(), interpolateQuaternions3[i].begin(), interpolateQuaternions3[i].end());
    data.insert(data.end(), interpolatedPoints3[i].begin(), interpolatedPoints3[i].end());
    concatenatedQuatPos.push_back(data);
  }

  std::vector<std::vector<double>> interpolatedPoints4 = interpolate2Points(p4, p5, n);
  std::vector<std::vector<double>> interpolateQuaternions4 = interpolateQuaternions(q4, q5, n);

  // Concatenate interpolated quaternions and points
  for (size_t i = 0; i < interpolatedPoints4.size(); ++i) {
    std::vector<double> data;
    data.insert(data.end(), interpolateQuaternions4[i].begin(), interpolateQuaternions4[i].end());
    data.insert(data.end(), interpolatedPoints4[i].begin(), interpolatedPoints4[i].end());
    concatenatedQuatPos.push_back(data);
  }

  return concatenatedQuatPos;
}
geometry_msgs::PoseStamped fillPoseStamped(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = "base_link";
  poseStamped.pose.position.x = position.x();
  poseStamped.pose.position.y = position.y();
  poseStamped.pose.position.z = position.z();
  poseStamped.pose.orientation.w = orientation.w();
  poseStamped.pose.orientation.x = orientation.x();
  poseStamped.pose.orientation.y = orientation.y();
  poseStamped.pose.orientation.z = orientation.z();
  return poseStamped;
}