cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(millimeter_wave_radar_test)

add_executable(${PROJECT_NAME} src/main.cc)


#target_include_directories(${PROJECT_NAME} PRIVATE /usr/include/linux)

