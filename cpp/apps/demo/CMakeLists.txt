add_executable(robotics_demo main.cpp)
target_link_libraries(robotics_demo PRIVATE robotics::robotics)
target_compile_options(robotics_demo PRIVATE -Wall -Wextra -Wpedantic)
