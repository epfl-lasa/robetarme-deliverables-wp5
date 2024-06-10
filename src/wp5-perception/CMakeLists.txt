cmake_minimum_required(VERSION 3.0.2)
project(wp5_perception)

add_definitions("-DWP5_PERCEPTION_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

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
  src/target_extraction.py
  src/point_cloud_transformer.py
  src/camera.sh
  src/target_substraction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
