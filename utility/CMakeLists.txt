cmake_minimum_required(VERSION 3.10)

project(utility)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_executable(csv src/use_csv.cc)

target_link_libraries(csv pthread)