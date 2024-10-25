cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 20)
set(LIBIGL_WITH_CGAL ON)
set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
# must added before add_executable and add_library...
add_compile_options(/bigobj)

project(marching_cubes)
add_executable(${PROJECT_NAME} main.cpp mc_lookup_tables.cpp marching_cubes.cpp)

# for unit test
enable_testing()
find_package(GTest CONFIG REQUIRED)
find_package(CGAL REQUIRED)
set(IGL_STATIC_LIBRARY_DIR "D:\\Library\\libigl\\build\\lib\\Debug")
add_executable(tests marching_cubes_test.cpp marching_cubes.cpp mc_lookup_tables.cpp)
target_link_libraries(tests GTest::gtest GTest::gmock)
target_link_libraries(tests CGAL::CGAL)
link_directories(IGL_STATIC_LIBRARY_DIR)
# target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/igl.lib)
# target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/igl_copyleft_cgal.lib)
# target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/igl_copyleft_core.lib)
# target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/igl_glfw.lib)
# target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/glad.lib)
# target_link_libraries(tests ${IGL_STATIC_LIBRARY_DIR}/glfw3.lib)
# target_compile_options(tests PRIVATE /bigobj)
add_test(AllTestsInMain tests)

include_directories("D:\\Library\\libigl\\build\\_deps\\eigen-src")
include_directories("D:\\Library\\libigl\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\glfw-src\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\glad-src\\include")
# add_compile_options(/Zm10000)
