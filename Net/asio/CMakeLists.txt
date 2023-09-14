cmake_minimum_required(VERSION 3.9)

project(tcp_server)

add_executable(tcp_server tcp_sync_server.cc)


find_package(Boost 1.55.0 REQUIRED 
             COMPONENTS system filesystem thread)
include_directories(tcp_server ${Boost_INCLUDE_DIRS})
link_directories(tcp_server ${Boost_LIBRARY_DIRS})
target_link_libraries(tcp_server ${Boost_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")