cmake_minimum_required(VERSION 3.9)

project(cfy_action)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs actionlib roscpp cfy_conn)
add_action_files(DIRECTORY src/action FILES FocusAdjust.action)
generate_messages(DEPENDENCIES actionlib_msgs)

add_executable(client src/action_client.cc)
add_executable(server src/action_server.cc src/mapping.h)
add_executable(test_server src/virtual_server.cc)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# connection
find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem thread)
link_directories(${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})


target_link_libraries(client ${catkin_LIBRARIES})
target_link_libraries(server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(test_server ${Boost_LIBRARIES})