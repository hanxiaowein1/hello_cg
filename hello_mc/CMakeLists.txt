cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 20)

project(marching_cubes)
add_executable(${PROJECT_NAME} main.cpp)

include_directories("D:\\Library\\libigl\\build\\_deps\\eigen-src")