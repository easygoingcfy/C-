cmake_minimum_required(VERSION 3.9)

project(single_server)
project(single_client)
project(multi_process_server)
project(multi_thread_server)

add_executable(single_server single_server.c)
add_executable(single_client single_client.c)
add_executable(multi_process_server multi_process_server.c)
add_executable(multi_thread_server multi_thread_server.c)
