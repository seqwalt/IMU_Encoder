cmake_minimum_required(VERSION 3.2)

project(ImuEKF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

# IMU EKF
add_library(imu_ekf ImuEKF.cpp)

# Arduino Eigen
add_library(ardu_eigen INTERFACE)
target_include_directories(ardu_eigen INTERFACE ${CMAKE_SOURCE_DIR}/../Bolder_Flight_Systems_Eigen/src/)

# Executable
add_executable(ImuEKF_example examples/cmake/main.cpp)
include_directories(${CMAKE_SOURCE_DIR}/../Bolder_Flight_Systems_Eigen/src)
target_link_libraries(ImuEKF_example
    ardu_eigen
    imu_ekf
)
