cmake_minimum_required(VERSION 3.10)
project(Robot-Arm-Modeling C)

# option to enable Debug mode for valgrind
# when compiling, add flag -DENABLE_DEBUG=ON
option(ENABLE_DEBUG "Enable Debug Mode" OFF)

if (ENABLE_DEBUG)
    message(STATUS "Debug mode enabled")
    set(CMAKE_BUILD_TYPE Debug)
    set(CMAKE_C_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
else()
    message(STATUS "Debug mode disabled (default Release mode)")
    set(CMAKE_BUILD_TYPE Release)
endif()


# Set the C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# Define the executable
add_executable(robot_arm_modeling
    # arm_modeling.c
    user_math.c
    unit_tests.c
)

# Include header files
target_include_directories(robot_arm_modeling PRIVATE ${CMAKE_SOURCE_DIR})

# Enable testing support
enable_testing()
add_test(NAME unit_tests COMMAND robot_arm_modeling)
