cmake_minimum_required(VERSION 3.10.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")

project(cgal_demo)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)

# add_subdirectory(stl_examples)
add_subdirectory(HalfedgeDS_examples)

add_subdirectory(hello_world)

add_subdirectory(surface_mesh)

add_subdirectory(polyhedron)