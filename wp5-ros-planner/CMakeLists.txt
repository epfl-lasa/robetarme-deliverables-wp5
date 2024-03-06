cmake_minimum_required(VERSION 3.3)
project(wp5-ros-planner)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  boustrophedon_msgs
  boustrophedon_server
  geometry_msgs
  roscpp
  roslib
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib boustrophedon_server boustrophedon_msgs geometry_msgs roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Create a shared library
add_library(${PROJECT_NAME}
  src/BoustrophedonServer.cpp
  src/PathPlanner.cpp
  src/TargetExtraction.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${roslib_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
