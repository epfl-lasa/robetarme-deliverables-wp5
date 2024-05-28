cmake_minimum_required(VERSION 3.0.2)
project(wp5_perception)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  message_filters
  std_msgs
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare catkin package
catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  src/camera_connection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
