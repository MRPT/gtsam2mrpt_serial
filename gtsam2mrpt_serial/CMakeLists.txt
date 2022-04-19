project(gtsam2mrpt_serial)

find_package(mrpt-serialization REQUIRED)
find_package(mrpt-io REQUIRED)
find_package(mrpt-poses REQUIRED)

find_package(GTSAM REQUIRED)


add_library(${PROJECT_NAME}
	serialize.cpp
	include/gtsam2mrpt_serial/serialize.h
)

#add_library(libs::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(${PROJECT_NAME}
	PRIVATE
		cxx_std_17
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
		mrpt::serialization
		mrpt::io
		mrpt::poses
		gtsam
)

if(BUILD_TESTING)
	add_subdirectory(tests)
endif()
