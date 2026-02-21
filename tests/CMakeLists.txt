add_executable(gtsam2mrpt_tests
    main.cpp
    sampleData.cpp
    sampleData.h
    boost-exports.h
)

# 3. Link against your library defined in the root CMake 
target_link_libraries(gtsam2mrpt_tests
    PRIVATE
        gtsam2mrpt_serial
)

# 4. Use the MODERN add_test signature
add_test(NAME gtsam2mrpt_serial_tests COMMAND gtsam2mrpt_tests)