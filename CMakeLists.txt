cmake_minimum_required(VERSION 3.0)
FIND_PACKAGE(jsoncpp REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})
include_directories(bullet3)

PROJECT(pybullet_cpp_IndyDualArm)

cmake_minimum_required(VERSION 3.12)

SUBDIRS(bullet3)
file(GLOB_RECURSE INDYDUALARM src/IndyDualArm.cpp src/IndyDualArm.h )
file(GLOB_RECURSE MR lib/ModernRobotics.cpp lib/ModernRobotics.h)
add_library(ModernRoboticsCpp SHARED include/ModernRobotics/modern_robotics.cpp include/ModernRobotics/modern_robotics.h)
add_library(INDYDUALARM SHARED include/IndyDualArm/IndyDualArm.cpp include/IndyDualArm/IndyDualArm.h include/ModernRobotics/modern_robotics.cpp include/ModernRobotics/modern_robotics.h )
add_executable(pybullet_cpp_IndyDualArm src/main.cpp )

target_link_libraries(pybullet_cpp_IndyDualArm  pybullet_cpp)
target_link_libraries( pybullet_cpp_IndyDualArm ModernRoboticsCpp )
target_link_libraries( pybullet_cpp_IndyDualArm INDYDUALARM )
target_link_libraries(pybullet_cpp_IndyDualArm  jsoncpp_lib)


