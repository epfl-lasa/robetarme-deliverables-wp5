cmake_minimum_required(VERSION 3.3)
project(wp5_tasks VERSION 0.1.0 DESCRIPTION "Base task library decriving the task to be performed.")

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(PYBIND11_PYTHON_VERSION "3.8")

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_TASKS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
add_definitions(-DPYBIND11_PYTHON_VERSION="${PYBIND11_PYTHON_VERSION}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5_ros_interface
  wp5_robotic_arms_drivers
  wp5_planner
  wp5_planner_ros
  wp5_dynamical_system
  wp5_tools
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(control_libraries 7.3.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pybind11 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  wp5_ros_interface
  wp5_robotic_arms_drivers
  wp5_planner
  wp5_planner_ros
  wp5_planner
  wp5_dynamical_system
  wp5_tools
  DEPENDS control_libraries pybind11 YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# Add the main node
set(node_name wp5_main_task_node)
add_executable(${node_name}
  src/main_task.cpp
  src/ITaskBase.cpp
  src/TaskShotcrete.cpp
  src/TaskSurfaceFinishing.cpp
)

target_link_libraries(${node_name} PRIVATE
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  pybind11::embed
)

# Add the main node
set(node_name_txt wp5_read_txt)
add_executable(${node_name_txt}
  src/ITaskBase.cpp
  src/TaskShotcrete.cpp
  src/TaskSurfaceFinishing.cpp
  src/savePathFromTxt.cpp
)


target_link_libraries(${node_name_txt} PRIVATE
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  pybind11::embed
)