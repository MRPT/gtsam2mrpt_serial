# 1. Find dependencies
find_package(GTSAM REQUIRED)
find_package(mrpt-serialization REQUIRED)
find_package(mrpt-io REQUIRED)
find_package(mrpt-math REQUIRED)
find_package(mrpt-random REQUIRED)
find_package(mrpt-poses REQUIRED)

# 2. Collect sources and public headers
# In your tree, sources are in the current dir and headers in include/ [cite: 1]
file(GLOB_RECURSE LIB_SRCS *.cpp)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

# 3. Define the library using MOLA macros
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
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

# Ensure C++17 as required by MRPT and this library
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 4. Define tests if enabled
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()