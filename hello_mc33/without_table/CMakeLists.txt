cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")

project(mc33_without_table)

find_package(Boost COMPONENTS functional REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} Boost:functional)

add_executable(tests mc33.cpp mc33_test.cpp)
enable_testing()
find_package(GTest CONFIG REQUIRED)
target_link_libraries(tests GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main Boost:functional)
add_test(AllTestsInMain tests)