cmake_minimum_required(VERSION 3.3)
project(wp5_planner)

add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(PYBIND11_PYTHON_VERSION "3.8")

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_PLANNER_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
add_definitions(-DPYBIND11_PYTHON_VERSION="${PYBIND11_PYTHON_VERSION}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  actionlib
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  polygon_coverage_planners
  polygon_coverage_msgs
  polygon_coverage_ros
)

# Find packages if any
find_package(Eigen3 3.3 REQUIRED)  # Ensure this version matches your system
find_package(yaml-cpp REQUIRED)
find_package(pybind11 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)  # Add this line to find PCL

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp actionlib geometry_msgs nav_msgs tf2_geometry_msgs polygon_coverage_planners polygon_coverage_msgs polygon_coverage_ros
  DEPENDS EIGEN3 YAML_CPP pybind11 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${pybind11_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  # Include Eigen3 headers
  ${PCL_INCLUDE_DIRS}    # Include PCL headers
)

# Create a shared library
add_library(${PROJECT_NAME} STATIC
  src/PolygonCoverage.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}  # Link PCL common libraries
  ${PCL_IO_LIBRARIES}      # Link PCL IO libraries
  Eigen3::Eigen
  pybind11::embed
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  src
  ${YAML_CPP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
