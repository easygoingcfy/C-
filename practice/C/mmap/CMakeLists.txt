cmake_minimum_required(VERSION 3.9)

project(read)
project(write)

add_executable(read mmap_read.c)

add_executable(write mmap_write.c)