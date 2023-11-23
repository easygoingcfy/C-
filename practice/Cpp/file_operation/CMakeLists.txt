cmake_minimum_required(VERSION 3.12)
project(file_operation)

find_package(Boost REQUIRED COMPONENTS filesystem system)


add_executable(${PROJECT_NAME} src/manager.cc)

target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})

