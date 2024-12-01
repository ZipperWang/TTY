cmake_minimum_required(VERSION 3.16.3)
project(TTY)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "/home/zipper/Documents/TTY")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "/home/zipper/Documents/TTY")
 
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories( include )

add_executable(test test.cpp)