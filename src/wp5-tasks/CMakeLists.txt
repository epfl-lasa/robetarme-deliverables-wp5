cmake_minimum_required(VERSION 3.3)
project(wp5-tasks)

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# Save the path during compilation to solve inclusion problems with linked files
add_definitions("-DWP5_TASKS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  wp5-ros-interface
  wp5-robotic-arms-drivers
  wp5-planner-ros
  wp5-dynamical-system
)

find_package(control_libraries 6.3.1 REQUIRED)

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
  src/main_task.cpp
  src/Tasks.cpp  # Include the source file of TargetExtraction if it's in a separate file

)

target_link_libraries(wp5-main-temp-node PRIVATE
  ${catkin_LIBRARIES}
  ${control_libraries_LIBRARIES}
)
