cmake_minimum_required(VERSION 3.9)

project(websocket_test)

set(CMAKE_BUILD_TYPE Debug)

find_package(Boost REQUIRED COMPONENTS system thread)

message("${project_name}")

add_executable(${PROJECT_NAME} src/test.cc src/client.cc src/server.cc )
add_executable(client src/my_client.cc src/client.cc )
add_executable(server src/my_server.cc src/server.cc)

include_directories(include ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(client ${Boost_LIBRARIES})
target_link_libraries(server ${Boost_LIBRARIES})
