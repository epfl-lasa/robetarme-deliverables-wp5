cmake_minimum_required(VERSION 3.3)
project(wp5-dynamical-system VERSION 0.1.0 DESCRIPTION "Drivers library to manage dynamical system")

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

add_definitions("-DWP5_DYNAMICAL_SYSTEM_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# Find packages if any
find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 3.4 REQUIRED)
find_package(yaml-cpp REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS EIGEN3 YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Create a static library
add_library(${PROJECT_NAME} STATIC
    src/DynamicalSystem.cpp
)

# Set lib version
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  src
  ${YAML_CPP_INCLUDE_DIR}
)

# Link library to robotic_arms_drivers
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
)

export(PACKAGE ${PROJECT_NAME})