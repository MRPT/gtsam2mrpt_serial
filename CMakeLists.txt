cmake_minimum_required(VERSION 3.10)

project(gtsam2mrpt_serial VERSION 0.0.1 LANGUAGES CXX)

include(CTest)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

add_subdirectory(gtsam2mrpt_serial)
#add_subdirectory(app)
