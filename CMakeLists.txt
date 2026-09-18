cmake_minimum_required(VERSION 3.10)
project(gtsam2mrpt_serial LANGUAGES CXX)

# Load MOLA CMake functions (mola_add_library, etc.)
find_package(mola_common REQUIRED)

# Standard CMake options
include(CTest)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Find dependencies
find_package(GTSAM REQUIRED)
find_package(mrpt_serialization REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_math REQUIRED)
find_package(mrpt_random REQUIRED)
find_package(mrpt_poses REQUIRED)

# Define the library using MOLA macros
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    src/serialize.cpp
    include/gtsam2mrpt_serial/serialize.h
  PUBLIC_LINK_LIBRARIES
    gtsam
    mrpt::mrpt_serialization
    mrpt::mrpt_io
    mrpt::mrpt_math
    mrpt::mrpt_random
    mrpt::mrpt_poses
  CMAKE_DEPENDENCIES
    mola_common
    mrpt_serialization
    mrpt_io
    mrpt_math
    mrpt_random
    mrpt_poses
)

# Define tests if enabled
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
