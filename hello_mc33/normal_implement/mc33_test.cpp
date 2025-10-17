#include "mc33_global_test.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    //::testing::GTEST_FLAG(filter) = "GlobalTest.vertex_interpolation_connected_cube_diagnal_connected";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.deserialize_sf_and_generate_mesh";
    ::testing::GTEST_FLAG(filter) = "GlobalTest.IGLBunny";
    // ::testing::GTEST_FLAG(filter) = "GlobalTest.get_edgess_const_vector_double_const_unordered_map";

    int result = RUN_ALL_TESTS();
    return result;
}