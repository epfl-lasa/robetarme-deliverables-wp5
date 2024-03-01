cmake_minimum_required(VERSION 3.3)
project(wp5-ros-interface)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Find the YAML_CPP library
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Create a shared library
add_library(${PROJECT_NAME}
  src/RosInterfaceNoetic.cpp
)

# Link the shared library against required libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
  ${roslib_LIBRARIES}
)

# Create an executable using the shared library
# add_executable(RosInterfaceNoeticNode src/RosInterfaceNoetic.cpp)
# target_link_libraries(RosInterfaceNoeticNode
#   ${PROJECT_NAME}  # Link against the shared library
# )

# Optionally, if you have a main.cpp file:
# add_executable(your_main_executable main.cpp)
# target_link_libraries(your_main_executable
#   ${PROJECT_NAME}  # Link against the shared library
# )
