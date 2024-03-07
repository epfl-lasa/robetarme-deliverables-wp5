cmake_minimum_required(VERSION 3.3)
project(wp5-main-temp)

# Add options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trac_ik_lib
  wp5-ros-interface
  wp5-robotic-arms-drivers
  wp5-dynamical-system
)
# find_package(  wp5-planner-ros-dependency REQUIRED)

find_package(control_libraries 6.3.1 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp wp5-ros-interface wp5-robotic-arms-drivers wp5-dynamical-system
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add the main_temp node
add_executable(wp5-main-temp-node
  src/main_temp.cpp
)
target_link_libraries(wp5-main-temp-node PRIVATE
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
)
