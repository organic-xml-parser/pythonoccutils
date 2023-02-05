cmake_minimum_required(VERSION 3.21)
project(PythonoccutilsSolvespaceWrapper)

add_executable(main src/main.cpp src/Allocator.cpp)

target_link_libraries(main slvs)
