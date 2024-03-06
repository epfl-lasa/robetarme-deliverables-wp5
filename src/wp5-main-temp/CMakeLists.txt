cmake_minimum_required(VERSION 3.3)
project(wp5-main-temp)

# Choose one of these options based on your needs
set(CMAKE_CXX_STANDARD 14)
# OR
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  trac_ik_lib
  wp5-ros-interface
)
# find_package(  wp5-planner-ros-dependency REQUIRED)

find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_libraries 6.3.1 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp wp5-ros-interface wp5-planner-ros
)

set(WP5_ROBOTIC_ARMS_DIR ${CMAKE_SOURCE_DIR}/wp5-robotic-arms)
set(WP5_DYNAMICAL_SYSTEM_DIR ${CMAKE_SOURCE_DIR}/wp5-dynamical-system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${WP5_ROBOTIC_ARMS_DIR}/include   
  ${WP5_DYNAMICAL_SYSTEM_DIR}/include   
  ${EIGEN3_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIRS}  
)

add_executable(wp5-main-temp-node src/main_temp.cpp)
target_link_libraries(wp5-main-temp-node
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
  ${WP5_ROBOTIC_ARMS_DIR}/build/librobotic_arms_drivers.a
  ${WP5_DYNAMICAL_SYSTEM_DIR}/build/libwp5-dynamical-system.a
  wp5-planner-ros 
  osqp::osqp
)
