add_executable(hello_world hello_world.cpp)

add_executable(draw_mesh draw_mesh.cpp)
target_link_libraries(draw_mesh ${IGL_STATIC_LIBRARY_DIR}/igl_glfw.lib)
target_link_libraries(draw_mesh ${IGL_STATIC_LIBRARY_DIR}/glad.lib)
target_link_libraries(draw_mesh ${IGL_STATIC_LIBRARY_DIR}/glfw3.lib)
target_link_libraries(draw_mesh ${IGL_STATIC_LIBRARY_DIR}/igl.lib)
