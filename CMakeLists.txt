cmake_minimum_required(VERSION 3.10)
project(gtsam2mrpt_serial LANGUAGES CXX)

# Load MOLA CMake functions (mola_add_library, etc.)
find_package(mola_common REQUIRED)

# Standard CMake options
include(CTest)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

add_subdirectory(gtsam2mrpt_serial)