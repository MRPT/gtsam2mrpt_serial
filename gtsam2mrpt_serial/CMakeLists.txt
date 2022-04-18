project(gtsam2mrpt_serial)

find_package(mrpt-serialization REQUIRED)
find_package(mrpt-io REQUIRED)

find_package(GTSAM REQUIRED)


add_library(${PROJECT_NAME}
	gtsam2mrpt_serial.cpp
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
		gtsam
)

if(BUILD_TESTING)
	add_subdirectory(tests)
endif()
