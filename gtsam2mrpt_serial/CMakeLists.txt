project(gtsam2mrpt_serial)

find_package(mrpt-serialization REQUIRED)
find_package(mrpt-io REQUIRED)
find_package(mrpt-poses REQUIRED)

if (NOT TARGET gtsam)
	find_package(GTSAM REQUIRED)
endif()

add_library(${PROJECT_NAME}
	serialize.cpp
	include/gtsam2mrpt_serial/serialize.h
)

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

# Add alias so it's defined exactly the same than if exported/imported via cmake
#
# Final lib name: g2m::gtsam2mrpt_serial
#
add_library(g2m::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

export(
  TARGETS ${PROJECT_NAME}
  FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}-targets.cmake
  NAMESPACE g2m::
)

install(TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}-targets
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION include/
)
install(EXPORT ${PROJECT_NAME}-targets
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake/
	NAMESPACE g2m::
)


if(BUILD_TESTING)
	add_subdirectory(tests)
endif()
