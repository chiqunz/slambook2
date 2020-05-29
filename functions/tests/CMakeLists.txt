cmake_minimum_required(VERSION 2.8)
project(tests)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

include_directories("/usr/include/eigen3")

add_executable(test_linalg test_linalg.cpp)
target_link_libraries(test_linalg PRIVATE LINALG)
