cmake_minimum_required(VERSION 3.3)
project(wp5_robotic_arms_drivers VERSION 0.1.0 DESCRIPTION "Drivers library to manage different robotic arms")

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(PYBIND11_PYTHON_VERSION "3.8")


# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_ROBOTIC_ARMS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

# Find packages if any
find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 3.4 REQUIRED)
find_package(control_libraries 7.3.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pybind11 REQUIRED)


## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS EIGEN3 control_libraries YAML_CPP pybind11
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
    src/IRoboticArmBase.cpp
    src/RoboticArmCobod.cpp
    src/RoboticArmCr7.cpp
    src/RoboticArmIiwa7.cpp
    src/RoboticArmUr5.cpp
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
  Eigen3::Eigen
  ${control_libraries_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

export(PACKAGE ${PROJECT_NAME})
