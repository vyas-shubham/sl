#############################################################################
#############################################################################
#
#  CMakeLists.txt for SL Examples
#
#  This CMakeLists.txt file builds examples demonstrating the SL library.
#  These examples require the SL library to be properly installed.
#
#  Copyright by Stefan Schaal, 2014
#  Extended for examples, 2024
#
#############################################################################
#############################################################################

cmake_minimum_required(VERSION 2.8)

#############################################################################
# include common cmake components

include($ENV{LAB_ROOT}/config/cmake/LAB.cmake)

#############################################################################
# Project settings

project(SL_Examples C)

# Build type
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Include directories
include_directories(BEFORE ../include)
include_directories(BEFORE $ENV{LAB_ROOT}/include)

# Link directories
link_directories($ENV{LAB_LIBDIR})

#############################################################################
# Example executables

# Example 1: Data Structures
add_executable(01_data_structures 01_data_structures.c)
target_link_libraries(01_data_structures SLcommon utility m)

# Example 2: Quaternion Operations
add_executable(02_quaternion_operations 02_quaternion_operations.c)
target_link_libraries(02_quaternion_operations SLcommon utility m)

# Example 3: Min-Jerk Trajectory
add_executable(03_min_jerk_trajectory 03_min_jerk_trajectory.c)
target_link_libraries(03_min_jerk_trajectory SLcommon SLtask utility m)

# Example 4: Signal Filtering
add_executable(04_signal_filtering 04_signal_filtering.c)
target_link_libraries(04_signal_filtering SLcommon utility m)

# Example 5: Real-Time IPC
add_executable(05_realtime_ipc 05_realtime_ipc.c)
target_link_libraries(05_realtime_ipc SLcommon utility pthread m)

#############################################################################
# Installation

install(TARGETS 
    01_data_structures 
    02_quaternion_operations
    03_min_jerk_trajectory
    04_signal_filtering
    05_realtime_ipc
    RUNTIME DESTINATION bin/examples
)
