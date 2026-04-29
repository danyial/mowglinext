# mowgli_geometry unit tests (Plan 01-02 Task 2).
# Header-only INTERFACE library — link via mowgli_geometry target so the
# transitive Eigen3 / Boost / geometry_msgs include directories are picked
# up automatically.

find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_offset_polygon_inward test_offset_polygon_inward.cpp)
target_link_libraries(test_offset_polygon_inward mowgli_geometry)
ament_target_dependencies(test_offset_polygon_inward geometry_msgs)

ament_add_gtest(test_footprint_check test_footprint_check.cpp)
target_link_libraries(test_footprint_check mowgli_geometry)
ament_target_dependencies(test_footprint_check geometry_msgs)

ament_add_gtest(test_pca_axis test_pca_axis.cpp)
target_link_libraries(test_pca_axis mowgli_geometry)
ament_target_dependencies(test_pca_axis geometry_msgs)

ament_add_gtest(test_atomic_write test_atomic_write.cpp)
target_link_libraries(test_atomic_write mowgli_geometry)

ament_add_gtest(test_key_value_parser test_key_value_parser.cpp)
target_link_libraries(test_key_value_parser mowgli_geometry)
