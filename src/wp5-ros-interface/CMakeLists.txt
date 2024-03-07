cmake_minimum_required(VERSION 3.3)
project(wp5-ros-interface VERSION 0.1.0 DESCRIPTION "Ros interface library to bind with specific ros version")

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs message_runtime sensor_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Create a static library
add_library(${PROJECT_NAME} STATIC
  src/RosInterfaceNoetic.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  src
)

# Link library to robotic_arms_drivers
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Link the shared library against required libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  yaml-cpp
)

export(PACKAGE ${PROJECT_NAME})

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
