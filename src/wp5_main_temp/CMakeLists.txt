cmake_minimum_required(VERSION 3.3)
project(wp5-main-temp)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5-ros-interface  # Add this line to find the wps-ros-interface package
)

catkin_package(
  CATKIN_DEPENDS roscpp wp5-ros-interface  # Add wps-ros-interface to dependencies
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wp5-main-temp-node src/main_temp.cpp)
target_link_libraries(wp5-main-temp-node ${catkin_LIBRARIES})
