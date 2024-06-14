cmake_minimum_required(VERSION 3.10)

project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/mnt/hgfs/ThirdParty/)

if(BUILD_TESTING)
    find_library(gtest_location NAMES libgtest.a)
    add_library(gtest STATIC IMPORTED)
    set_target_properties(gtest PROPERTIES IMPORTED_LOCATION ${gtest_location})
    message(STATUS ${gtest_location})
    add_definitions(-DGTEST_ENABLED=1)
    add_executable(${PROJECT_NAME} rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} gtest)
else()
    add_executable(${PROJECT_NAME} main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
endif()

