cmake_minimum_required(VERSION 3.10)
project(Robot-Arm-Modeling C)

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
