cmake_minimum_required(VERSION 3.3)
project(wp5_tools VERSION 0.1.0 DESCRIPTION "Drivers library to manage different robotic arms")

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_TOOLS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Find packages if any
find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 3.4 REQUIRED)
find_package(yaml-cpp REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs geometry_msgs
  DEPENDS EIGEN3  YAML_CPP
)

include_directories(
  include
  urdf
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Create a STATIC library instead of SHARED
add_library(${PROJECT_NAME} STATIC
    src/IToolsBase.cpp
    src/ToolsShotcrete.cpp
    src/ToolsSurfaceFinishing.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  urdf
  src
  ${YAML_CPP_INCLUDE_DIR}
)

# Link library to robotic_arms_drivers
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
)

export(PACKAGE ${PROJECT_NAME})
