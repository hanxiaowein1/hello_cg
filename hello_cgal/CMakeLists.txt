cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")

project(cgal_demo)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)

add_subdirectory(stl_examples)

