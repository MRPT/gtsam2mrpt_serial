project(gtsam2mrpt_tests)

add_executable(${PROJECT_NAME}
    main.cpp
    sampleData.cpp
    sampleData.h
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        gtsam2mrpt_serial
)

add_test(gtsam2mrpt_serial_tests ${PROJECT_NAME})
