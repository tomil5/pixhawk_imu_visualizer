cmake_minimum_required(VERSION 3.10)
project(PixhawkIMUVisualizer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Используем локальные MAVLink заголовки
include_directories(${CMAKE_SOURCE_DIR}/c_library_v2)

add_executable(pixhawk_imu_reader src/pixhawk_imu_reader.cpp)
target_link_libraries(pixhawk_imu_reader pthread)