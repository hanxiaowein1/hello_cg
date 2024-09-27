add_executable(three_points_one_segment three_points_one_segment.cpp)
target_link_libraries(three_points_one_segment CGAL::CGAL)

add_executable(suprising surprising.cpp)
target_link_libraries(suprising CGAL::CGAL)

add_executable(exact exact.cpp)
target_link_libraries(exact CGAL::CGAL)

add_executable(array_convex_hull_2 array_convex_hull_2.cpp)
target_link_libraries(array_convex_hull_2 CGAL::CGAL)

add_executable(convex_hull_yz convex_hull_yz.cpp)
target_link_libraries(convex_hull_yz CGAL::CGAL)
