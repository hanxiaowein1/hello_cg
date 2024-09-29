add_executable(check_orientation check_orientation.cpp)
target_link_libraries(check_orientation CGAL::CGAL)
target_link_libraries(check_orientation CGAL::CGAL_Basic_viewer)

add_executable(sm_points sm_points.cpp)
target_link_libraries(sm_points CGAL::CGAL)
target_link_libraries(sm_points CGAL::CGAL_Basic_viewer)

add_executable(sm_iterators sm_iterators.cpp)
target_link_libraries(sm_iterators CGAL::CGAL)
target_link_libraries(sm_iterators CGAL::CGAL_Basic_viewer)

add_executable(sm_circulators sm_circulators.cpp)
target_link_libraries(sm_circulators CGAL::CGAL)
target_link_libraries(sm_circulators CGAL::CGAL_Basic_viewer)

add_executable(sm_properties sm_properties.cpp)
target_link_libraries(sm_properties CGAL::CGAL)
target_link_libraries(sm_properties CGAL::CGAL_Basic_viewer)

add_executable(sm_kruskal sm_kruskal.cpp)
target_link_libraries(sm_kruskal CGAL::CGAL)
target_link_libraries(sm_kruskal CGAL::CGAL_Basic_viewer)

add_executable(sm_bgl sm_bgl.cpp)
target_link_libraries(sm_bgl CGAL::CGAL)
target_link_libraries(sm_bgl CGAL::CGAL_Basic_viewer)

add_executable(sm_memory sm_memory.cpp)
target_link_libraries(sm_memory CGAL::CGAL)
target_link_libraries(sm_memory CGAL::CGAL_Basic_viewer)