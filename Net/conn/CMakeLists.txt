cmake_minimum_required(VERSION 3.9)

project(client)

add_executable(client test.cc tcp.cpp interface.cpp)

find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(client ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
