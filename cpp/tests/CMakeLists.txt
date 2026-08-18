find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(robotics_tests
    test_serial_chain.cpp
    test_models.cpp
    test_solvers.cpp
    test_robot_registry.cpp
)
target_link_libraries(robotics_tests PRIVATE robotics::robotics GTest::gtest GTest::gtest_main)
target_compile_options(robotics_tests PRIVATE -Wall -Wextra -Wpedantic)

gtest_discover_tests(robotics_tests)
