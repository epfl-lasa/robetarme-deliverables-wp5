cmake_minimum_required(VERSION 3.3)
project(wp5_planner_ros)

add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_PLANNER_ROS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  actionlib
  geometry_msgs
  boustrophedon_msgs
  boustrophedon_server
)
find_package(yaml-cpp REQUIRED)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp actionlib geometry_msgs boustrophedon_msgs boustrophedon_server 
  DEPENDS YAML_CPP
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Create a shared library
add_library(${PROJECT_NAME} STATIC
  src/PathPlanner.cpp
  src/TargetExtraction.cpp
  src/BoustrophedonServer.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${boustrophedon_server_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  src
  ${YAML_CPP_INCLUDE_DIR}
)
