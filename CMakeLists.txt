cmake_minimum_required(VERSION 3.10)
project(gtsam2mrpt_serial LANGUAGES CXX)

# Load MOLA CMake functions (mola_add_library, etc.)
find_package(mola_common REQUIRED)

# Standard CMake options
include(CTest)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Find dependencies
find_package(GTSAM REQUIRED)
find_package(mrpt-serialization REQUIRED)
find_package(mrpt-io REQUIRED)
find_package(mrpt-math REQUIRED)
find_package(mrpt-random REQUIRED)
find_package(mrpt-poses REQUIRED)

# Define the library using MOLA macros
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    src/serialize.cpp
    include/gtsam2mrpt_serial/serialize.h
  PUBLIC_LINK_LIBRARIES
    gtsam
    mrpt::serialization
    mrpt::io
    mrpt::math
    mrpt::random
    mrpt::poses
  CMAKE_DEPENDENCIES
    mola_common
    mrpt-serialization
    mrpt-io
    mrpt-math
    mrpt-random
    mrpt-poses
)

# Define tests if enabled
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
