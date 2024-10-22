add_executable(signed_distance signed_distance.cpp)

target_link_libraries(signed_distance ${IGL_STATIC_LIBRARY_DIR}/igl_glfw.lib)
target_link_libraries(signed_distance ${IGL_STATIC_LIBRARY_DIR}/glad.lib)
target_link_libraries(signed_distance ${IGL_STATIC_LIBRARY_DIR}/glfw3.lib)
target_link_libraries(signed_distance ${IGL_STATIC_LIBRARY_DIR}/igl.lib)
