cmake_minimum_required(VERSION 3.0.2)
project(wp5-dynamical-system VERSION 0.1.0 DESCRIPTION "Drivers library to manage dynamical system")
cmake_policy(SET CMP0057 NEW)

# Add options
# add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 14) # specify the C++ standard
set(CMAKE_BUILD_TYPE Debug)

# Find packages if any
find_package(Eigen3 3.4 REQUIRED)


# # trac-ik 
# find_package(catkin REQUIRED COMPONENTS
#     trac_ik_lib)

include_directories(
  include
#   ${catkin_INCLUDE_DIRS}	
  ${EIGEN3_INCLUDE_DIRS}
)

# Include source file for the target library
set(lib_name wp5-dynamical-system)

# Create a STATIC library instead of SHARED
add_library(${lib_name} STATIC
    src/DynamicalSystem.cpp
)

# Set lib version
set_target_properties(${lib_name} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${lib_name} PRIVATE include)
target_include_directories(${lib_name} PRIVATE src)

# Link library to robotic_arms_drivers
target_link_libraries(${lib_name}
    ${control_libraries_LIBRARIES}
    Eigen3::Eigen

)

export(PACKAGE ${PROJECT_NAME})
