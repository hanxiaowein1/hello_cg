cmake_minimum_required(VERSION 3.15.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")

set(IGL_STATIC_LIBRARY_DIR "D:\\Library\\libigl\\build\\lib\\Debug")

project(hello_mc33)

file(GLOB hello_mc33_src src/hello_world.cpp src/chaos.cpp)

add_executable(${PROJECT_NAME} ${hello_mc33_src})
find_package(GLUT)
# find_package(glad)
# find_package(glfw3)
# target_link_libraries(${PROJECT_NAME} glad::glad GLUT::GLUT glfw)
target_link_libraries(${PROJECT_NAME} GLUT::GLUT)
target_link_libraries(${PROJECT_NAME} ${IGL_STATIC_LIBRARY_DIR}/igl_glfw.lib)
target_link_libraries(${PROJECT_NAME} ${IGL_STATIC_LIBRARY_DIR}/glad.lib)
target_link_libraries(${PROJECT_NAME} ${IGL_STATIC_LIBRARY_DIR}/glfw3.lib)
target_link_libraries(${PROJECT_NAME} ${IGL_STATIC_LIBRARY_DIR}/igl.lib)

enable_testing()
find_package(GTest CONFIG REQUIRED)
add_executable(tests src/mc33_test.cpp)
target_link_libraries(tests GTest::gtest GTest::gmock)
target_link_libraries(tests GLUT::GLUT)
target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/igl_glfw.lib)
target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/glad.lib)
target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/glfw3.lib)
target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/igl.lib)
add_test(AllTestsInMain tests)

include_directories("D:\\Library\\libigl\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\eigen-src")
include_directories("D:\\Library\\libigl\\build\\_deps\\glfw-src\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\glad-src\\include")
add_compile_options(/bigobj)
