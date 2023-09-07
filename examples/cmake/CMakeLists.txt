cmake_minimum_required(VERSION 3.2)

project(EKF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

# IMU EKF
add_library(EKF ../../ImuEncEKF.cpp)

# Arduino BasicLinearAlgebra
add_library(BLA INTERFACE)
target_include_directories(BLA INTERFACE ${CMAKE_SOURCE_DIR}/../../../BasicLinearAlgebra)

# Executable
add_executable(EKF_example main.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/../../../BasicLinearAlgebra
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/..
)
target_link_libraries(EKF_example
    BLA
    EKF
)
