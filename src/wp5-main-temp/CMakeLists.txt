cmake_minimum_required(VERSION 3.3)
project(wp5_main_temp)

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5_ros_interface
  wp5_robotic_arms_drivers
  wp5_planner_ros
  wp5_dynamical_system
)

find_package(control_libraries 7.3.0 REQUIRED)

catkin_package(CATKIN_DEPENDS
  roscpp
  wp5_ros_interface
  wp5_robotic_arms_drivers
  wp5_planner_ros
  wp5_dynamical_system
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the main_temp node
add_executable(${PROJECT_NAME}_node
  src/main_temp.cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
)
