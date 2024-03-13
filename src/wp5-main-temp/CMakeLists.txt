cmake_minimum_required(VERSION 3.3)
project(wp5-main-temp)

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5-ros-interface
  wp5-robotic-arms-drivers
  wp5-planner-ros
  wp5-dynamical-system
)

find_package(control_libraries 7.3.0 REQUIRED)

catkin_package(CATKIN_DEPENDS
  roscpp
  wp5-ros-interface
  wp5-robotic-arms-drivers
  wp5-planner-ros
  wp5-dynamical-system
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the main_temp node
add_executable(wp5-main-temp-node
  src/main_temp.cpp
)

target_link_libraries(wp5-main-temp-node PRIVATE
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
)
