cmake_minimum_required(VERSION 3.9)

project(websocket_test)

set(CMAKE_BUILD_TYPE Debug)

find_package(Boost REQUIRED COMPONENTS system thread)

# gen proto
find_package(Protobuf REQUIRED)

set(PBF_SRC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/generated)

set(PROTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/proto)
# Genenate aw_pbf src files
file(GLOB proto_files ${PROTOS_DIR}/*.proto)
foreach(proto_file ${proto_files})
  get_filename_component(FILE_NAME_WE ${proto_file} NAME_WE)
  list(APPEND PROTO_SRCS "${PBF_SRC_OUT_DIR}/${FILE_NAME_WE}.pb.cc")
  add_custom_command(
    OUTPUT "${PBF_SRC_OUT_DIR}/${FILE_NAME_WE}.pb.h"
           "${PBF_SRC_OUT_DIR}/${FILE_NAME_WE}.pb.cc"
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} -I=${PROTOS_DIR}
            --cpp_out=${PBF_SRC_OUT_DIR} ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Compiling ${proto_file}")
endforeach(proto_file)

set_source_files_properties(${PROTO_SRCS} PROPERTIES GENERATED TRUE)

include_directories(include ${Boost_INCLUDE_DIRS} ${PBF_SRC_OUT_DIR})
message("${project_name}")

add_executable(${PROJECT_NAME} src/test.cc src/client.cc src/server.cc
                               src/util.cc ${PROTO_SRCS})
add_executable(client src/my_client.cc src/client.cc src/util.cc ${PROTO_SRCS})
add_executable(server src/my_server.cc src/server.cc src/util.cc ${PROTO_SRCS})

link_directories(${Boost_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Protobuf_LIBRARIES})
target_link_libraries(client ${Boost_LIBRARIES} ${Protobuf_LIBRARIES})
target_link_libraries(server ${Boost_LIBRARIES} ${Protobuf_LIBRARIES})
