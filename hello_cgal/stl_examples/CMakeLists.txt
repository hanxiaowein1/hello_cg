
add_executable(stl_viewer stl_viewer.cpp)

target_link_libraries(stl_viewer CGAL::CGAL)
target_link_libraries(stl_viewer CGAL::CGAL_Basic_viewer)