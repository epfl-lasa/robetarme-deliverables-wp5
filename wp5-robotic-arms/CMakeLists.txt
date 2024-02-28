cmake_minimum_required(VERSION 3.0.2)
project(robotic_arms_drivers VERSION 0.1.0 DESCRIPTION "Drivers library to manage the different robotic arms")

# Add options
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17) # specify the C++ standard

# Find packages if any
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include source file for the target library
set(lib_name robotic_arms_drivers)
add_library(${lib_name}
    src/RoboticArmCobod.cpp
    src/RoboticArmCr7.cpp
    src/RoboticArmIiwa7.cpp
    src/RoboticArmUr5.cpp
)

# Set lib version
set_target_properties(${lib_name} PROPERTIES
    VERSION ${PROJECT_VERSION}
)

# Add include and src to include path to make includes easier in the code
target_include_directories(${lib_name} PRIVATE include)
target_include_directories(${lib_name} PRIVATE src)
target_link_libraries(${lib_name} PRIVATE Eigen3::Eigen yaml-cpp)

export(PACKAGE ${PROJECT_NAME})
