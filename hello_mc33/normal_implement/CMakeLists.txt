cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 20)
set(LIBIGL_WITH_CGAL ON)
set(CMAKE_TOOLCHAIN_FILE "D:\\Library\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
option(BUILD_MC33_LIB "enable building marching cubes 33 library" OFF)
option(BUILD_MC33_UNIT_TEST "enable building marching cubes 33 unit test" ON)
# add_compile_options must added before add_executable and add_library...
add_compile_options(/bigobj)

project(marching_cubes_33)
find_package(GTest CONFIG REQUIRED)
find_package(Boost COMPONENTS functional REQUIRED)
# # useless
# add_executable(${PROJECT_NAME} main.cpp mc33.cpp mc33_lookup_table.cpp)
# target_link_libraries(${PROJECT_NAME} Boost::functional)
# target_link_libraries(${PROJECT_NAME} GTest::gtest GTest::gmock)

function(generate_proto_files PROTO_FILES OUT_HDRS OUT_SRCS)
    set(GENERATED_HEADERS "")
    set(GENERATED_SOURCES "")

    foreach(PROTO_FILE IN LISTS PROTO_FILES)
        message(STATUS "current generating proto file ${PROTO_FILE}")
        # Get the base name of the proto file (e.g., 'messages')
        get_filename_component(PROTO_NAME "${PROTO_FILE}" NAME_WE)
        
        # Calculate the expected output file paths
        set(HEADER_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h")
        set(SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc")
        
        # Add to the output lists
        list(APPEND GENERATED_HEADERS "${HEADER_FILE}")
        list(APPEND GENERATED_SOURCES "${SOURCE_FILE}")
        execute_process(
            COMMAND ${Protobuf_PROTOC_EXECUTABLE}
                    --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
                    -I ${CMAKE_CURRENT_SOURCE_DIR}/proto
                    ${PROTO_FILE}
            RESULT_VARIABLE _proto_result
        )
        if(NOT _proto_result EQUAL 0)
            message(FATAL_ERROR "Failed to generate protobuf files from ${PROTO_FILE}")
        endif()

    endforeach()

    set(${OUT_HDRS} ${GENERATED_HEADERS} PARENT_SCOPE)
    set(${OUT_SRCS} ${GENERATED_SOURCES} PARENT_SCOPE)
endfunction()

# for unit test
if(BUILD_MC33_UNIT_TEST)
    enable_testing()
    find_package(CGAL REQUIRED)
    find_package(Protobuf CONFIG REQUIRED)
    set(IGL_STATIC_LIBRARY_DIR "D:\\Library\\libigl\\build\\lib\\Debug")
    set(
        MY_PROTO_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/tensor_flat.proto
    )
    generate_proto_files(
        "${MY_PROTO_FILES}"
        PROTO_H_FILES
        PROTO_CC_FILES
    )
    add_executable(
        tests
        mc33_test.cpp
        mc33.cpp
        mc33_lookup_table.cpp
        signed_distances_iface.cpp
        charles_mc33_type.cpp
        mc33_cache.cpp
        ${PROTO_CC_FILES}
    )
    target_link_libraries(tests GTest::gtest GTest::gmock)
    target_link_libraries(tests CGAL::CGAL)
    target_link_libraries(tests Boost::functional)
    target_link_libraries(tests protobuf::libprotobuf)
    target_include_directories(tests PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    link_directories(IGL_STATIC_LIBRARY_DIR)
    add_test(AllTestsInMain tests)
endif()

if(BUILD_MC33_LIB)
    find_package(Protobuf CONFIG REQUIRED)
    set(
        MY_PROTO_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/tensor_flat.proto
    )

    generate_proto_files(
        "${MY_PROTO_FILES}"
        PROTO_H_FILES
        PROTO_CC_FILES
    )
    message(STATUS "Generated sources: ${PROTO_CC_FILES}")
    message(STATUS "Generated headers: ${PROTO_H_FILES}")

    add_library(
        ${PROJECT_NAME}
        mc33.cpp
        mc33_lookup_table.cpp
        signed_distances_iface.cpp
        ${PROTO_CC_FILES}
    )
    target_link_libraries(${PROJECT_NAME} GTest::gtest GTest::gmock CGAL::CGAL Boost::functional protobuf::libprotobuf)
    target_link_directories(${PROJECT_NAME} IGL_STATIC_LIBRARY_DIR)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

    # 引入要用到的 CMake 模块
    include(CMakePackageConfigHelpers)
    include(GNUInstallDirs)

    # 基本安装及 Targets 文件的生成
    install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # Targets 文件的安装
    install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}-targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

    # Config 文件的导出（与安装？）
    configure_package_config_file(
        ${PROJECT_SOURCE_DIR}/cmake/config.cmake.in
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

    # ConfigVersion 文件的导出
    write_basic_package_version_file(
        ${PROJECT_NAME}-config-version.cmake
        VERSION 0.1
        COMPATIBILITY AnyNewerVersion
    )

    # Config 和 ConfigVersion 文件的安装
    install(FILES
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )

    install(FILES ${PROTO_H_FILES} DESTINATION include/${PROJECT_NAME})
endif()


include_directories("D:\\Library\\libigl\\build\\_deps\\eigen-src")
include_directories("D:\\Library\\libigl\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\glfw-src\\include")
include_directories("D:\\Library\\libigl\\build\\_deps\\glad-src\\include")
