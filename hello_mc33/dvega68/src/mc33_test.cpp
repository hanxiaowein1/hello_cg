#include <iostream>
#include "gtest/gtest.h"
#include "grid3d.cpp"
#include "mc33.cpp"
#include "surface.cpp"

class GlobalTest
{
    ~GlobalTest() = default;
    void SetUp() {};
    void TearDown() {};
};

TEST(GlobalTest, DrawBunny)
{
    grid3d o_grid3d;
	o_grid3d.init_with_bunny("D:\\Library\\libigl\\build\\_deps\\libigl_tutorial_tata-src\\bunny.off", 50, 50, 50);
	//o_grid3d.set_sphere(3, 3, 3, 0.8f);

	MC33 o_mc33;
	o_mc33.set_grid3d(o_grid3d);

	surface o_surface;
	o_mc33.calculate_isosurface(o_surface, 0);

	o_surface.save_as_obj("bunny.obj");
    ASSERT_EQ(true, true);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // Custom code before running tests
    std::cout << "Before running tests." << std::endl;

    int result = RUN_ALL_TESTS();

    // Custom code after running tests
    std::cout << "After running tests." << std::endl;

    return result;
}