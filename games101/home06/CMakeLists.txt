cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

if(BUILD_TESTING)
    find_library(gtest_location NAMES libgtest.a)
    add_library(gtest STATIC IMPORTED)
    set_target_properties(gtest PROPERTIES IMPORTED_LOCATION ${gtest_location})
    message(STATUS ${gtest_location})
    add_definitions(-DGTEST_ENABLED=1)
    add_executable(unit_test unit_test.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
    Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
    Renderer.cpp Renderer.hpp)
    target_link_libraries(unit_test ${OpenCV_LIBRARIES} gtest)
else()
    add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
                Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
                Renderer.cpp Renderer.hpp)
endif()
