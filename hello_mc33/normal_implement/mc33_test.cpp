#include "mc33_global_test.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    //::testing::GTEST_FLAG(filter) = "GlobalTest.vertex_interpolation_connected_cube_diagnal_connected";
    ::testing::GTEST_FLAG(filter) = "GlobalTest.has_value_bigger_than_zero_in_interval";
    int result = RUN_ALL_TESTS();
    return result;
}