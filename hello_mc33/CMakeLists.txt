cmake_minimum_required(VERSION 3.15.0)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")

project(hello_mc33)

file(GLOB hello_mc33_src src/hello_world.cpp src/chaos.cpp)

add_executable(${PROJECT_NAME} ${hello_mc33_src})

find_package(glad)
find_package(GLUT)
find_package(glfw3)

target_link_libraries(${PROJECT_NAME} glad::glad GLUT::GLUT glfw)
