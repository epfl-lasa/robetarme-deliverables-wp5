cmake_minimum_required(VERSION 3.3)
project(wp5_calibration VERSION 0.1.0 DESCRIPTION "Base task library decriving the task to be performed.")

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_CALIBRATION_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5_ros_interface
  wp5_robotic_arms_drivers
  wp5_planner_ros
  wp5_dynamical_system
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(control_libraries 7.3.0 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  wp5_ros_interface
  wp5_robotic_arms_drivers
  wp5_planner_ros
  wp5_dynamical_system
  DEPENDS control_libraries YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Add the main_temp node
add_executable(wp5_calibration_node
  src/main_calibration.cpp
)
target_link_libraries(wp5_calibration_node PRIVATE
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
# Add the main_temp node
add_executable(wp5_calibration_plateform_node
src/main_calibration_plateform.cpp
)
target_link_libraries(wp5_calibration_plateform_node PRIVATE
${catkin_LIBRARIES}
${control_libraries_LIBRARIES}
${Boost_LIBRARIES}
${YAML_CPP_LIBRARIES}
)

# Add the main_temp node
add_executable(wp5_calibration_camera_node
src/main_calibration_camera.cpp
)
target_link_libraries(wp5_calibration_camera_node PRIVATE
${catkin_LIBRARIES}
${control_libraries_LIBRARIES}
${Boost_LIBRARIES}
${YAML_CPP_LIBRARIES}
)


# Add the main_temp node
add_executable(wp5_calibration_camera_joint_node
src/main_calibration_camera_jointspace.cpp
)
target_link_libraries(wp5_calibration_camera_joint_node PRIVATE
${catkin_LIBRARIES}
${control_libraries_LIBRARIES}
${Boost_LIBRARIES}
${YAML_CPP_LIBRARIES}
)





