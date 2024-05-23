#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief A class managing the Dynamical System to calculate the desired velocity based on end-effector pose and path.
 */
class DynamicalSystem {
public:
  /**
   * @brief Constructs a new DynamicalSystem object.
   * @param freq The frequency of the system.
   */
  DynamicalSystem(double freq);

  /**
   * @brief Initializes the parameters of the Dynamical System.
   */
  void parameterInitialization();

  /**
   * @brief Sets the path for the Dynamical System.
   * @param firstQuatPos The initial quaternion positions of the path.
   */
  void setPath(std::vector<std::vector<double>> firstQuatPos);

  /**
   * @brief Sets the Cartesian pose of the end effector.
   * @param pose A pair containing the quaternion orientation and position vector.
   */
  void setCartPose(std::pair<Eigen::Quaterniond, Eigen::Vector3d> pose);

  /**
   * @brief Sets the linear speed of the Dynamical System.
   * @param speed The linear speed.
   */
  void setLinearSpeed(double speed);

  /**
   * @brief Sets the convergence rate and speed of the Limit Cycle.
   * @param angSpeed The angular speed.
   * @param conv The convergence rate.
   */
  void setLimitCycleSpeedConv(double angSpeed, double conv);

  /**
   * @brief Sets the radius of the Limit Cycle.
   * @param rad The radius of the Limit Cycle.
   */
  void setLimitCycleRadius(double rad);

  /**
   * @brief Sets the offset from the target
   * @param offset The offset
   */
  void setToolOffsetFromTarget_(double offset);

  /**
   * @brief Sets the tolerance to the next point in the path.
   * @param tol The tolerance value.
   */
  void setToleranceNextPoint(double tol);

  /**
   * @brief Checks if the Dynamical System has finished.
   * @return True if finished, false otherwise.
   */
  bool isFinished() const;

  /**
   * @brief Checks if the Dynamical System is initialized.
   * @return True if initialized, false otherwise.
   */
  bool isInitialized() const;

  /**
   * @brief Checks if the Dynamical System follows a linear trajectory.
   * @return True if following linear trajectory, false otherwise.
   */
  bool checkLinearDs() const;

  /**
   * @brief Calculates the desired quaternion and position for a given quaternion position.
   * @param desiredQuatPos The desired quaternion position.
   * @return A pair containing the desired quaternion and position.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getLinearDsOnePosition(std::vector<double> desiredQuatPos);

  /**
   * @brief Calculates the desired quaternion and position velocity.
   * @return A pair containing the desired quaternion velocity and position.
   */
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getDsQuatSpeed();

  /**
   * @brief Gets the initial quaternion positions of the path.
   * @return The initial quaternion positions.
   */
  std::vector<double> getFirstQuatPos() const;

  /**
   * @brief Calculates the twist from the Dynamical System.
   * @param quat1 The initial quaternion.
   * @param pairQuatPos A pair containing the quaternion and position.
   * @return The twist vector.
   */
  Eigen::VectorXd getTwistFromDS(Eigen::Quaterniond quat1, std::pair<Eigen::Quaterniond, Eigen::Vector3d> pairQuatPos);

  /**
   * @brief Updates the position and velocity of the Limit Cycle in 3D.
   * @param pose The current pose.
   * @param poseTarget The target pose.
   * @return The updated position and velocity.
   */
  Eigen::Vector3d updateLimitCycle3DPosVelWith2DLC(Eigen::Vector3d pose, Eigen::Vector3d poseTarget);

  /**
   * @brief Resets the initialization state.
   */
  void resetInit();

  /**
   * @brief Resets the linear Dynamical System state.
   */
  void resetCheckLinearDs();

  /**
   * @brief Restarts the path.
   */
  void restartPath();

  /**
   * @brief Adds an offset to the desired quaternion position.
   * @param desiredQuatPos The desired quaternion position.
   * @return The offset quaternion position.
   */
  std::vector<double> addOffset(std::vector<double> desiredQuatPos);

private:
  bool finish_ = false;
  bool init_ = false;

  bool checkLinearDs_ = false;
  Eigen::Vector3d pathPoint_;

  double convergenceRateLC_ = 10;
  double cycleRadiusLC_ = 0.03;
  double cycleSpeedLC_ = 2.5 * 3.14;
  double fs_ = 100;
  double toleranceToNextPoint_ = 0.1;
  double linearVelExpected_ = 0.04;

  std::size_t iFollow_ = 0;

  std::vector<std::vector<double>> desiredPath_;
  std::vector<double> firstQuatPos_;
  std::vector<double> lastQuatPos_;
  Eigen::Vector3d desiredVel_;

  Eigen::Vector3d realPos_;
  Eigen::Quaterniond realQuat_;

  Eigen::Vector3d centerLimitCycle_;
  Eigen::Vector4d desiredOriVelocityFiltered_;

  double toolOffsetFromTarget_, velocityLimit_;

  Eigen::Matrix<double, 4, 1> slerpQuaternion_(Eigen::Matrix<double, 4, 1>& q1,
                                               Eigen::Matrix<double, 4, 1>& q2,
                                               double t);

  Eigen::Matrix<double, 4, 1> quaternionProduct_(Eigen::Matrix<double, 4, 1> q1, Eigen::Matrix<double, 4, 1> q2);
};
