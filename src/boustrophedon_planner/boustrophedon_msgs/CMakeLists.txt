cmake_minimum_required(VERSION 2.8.3)
project(boustrophedon_msgs)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  genmsg
  std_msgs
  geometry_msgs
  actionlib_msgs
  nav_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  StripingPoint.msg
  StripingPlan.msg
)

add_service_files(
  FILES
  ConvertPlanToPath.srv
)

add_action_files(
  DIRECTORY action
  FILES
  PlanMowingPath.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  nav_msgs
)

catkin_package()
