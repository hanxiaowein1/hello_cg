cmake_minimum_required(VERSION 3.10.0)

option(LIBIGL_USE_STATIC_LIBRARY "Use libIGL as static librarie" ON)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE debug)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
# set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
add_compile_options(/bigobj)

project(hello_libigl)
set(IGL_STATIC_LIBRARY_DIR "D:\\Library\\libigl\\build\\lib\\Debug")

include_directories("D:\\Library\\libigl\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\eigen-src")
include_directories("D:\\Library\\libigl\\build\\_deps\\glfw-src\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\glad-src\\include")

add_subdirectory(hello_world)
