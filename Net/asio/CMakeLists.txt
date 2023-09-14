cmake_minimum_required(VERSION 3.9)

project(tcp)

add_executable(tcp_server tcp_sync_server.cc)
add_executable(tcp_client tcp_sync_client.cc)


find_package(Boost 1.55.0 REQUIRED 
             COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(tcp_server ${Boost_LIBRARIES})
target_link_libraries(tcp_client ${Boost_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")