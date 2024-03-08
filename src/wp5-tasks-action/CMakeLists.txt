cmake_minimum_required(VERSION 3.3)
project(wp5_tasks_action)

add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  std_msgs
  message_generation
  message_runtime
)

add_action_files(
  DIRECTORY action
  FILES ExecuteTask.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(CATKIN_DEPENDS
  roscpp
  actionlib
  actionlib_msgs
  std_msgs
  message_runtime
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the main node
add_executable(${PROJECT_NAME}_node
  src/tasks_action_server.cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE
  ${catkin_LIBRARIES}
)

# add_dependencies(
#   ${PROJECT_NAME}_node
#   ${actionlib_EXPORTED_TARGETS}
# )
