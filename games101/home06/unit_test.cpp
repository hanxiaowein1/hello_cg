#include "Bounds3.hpp"
#include "gtest/gtest.h"
#include "Triangle.hpp"

class GlobalTest
{
    ~GlobalTest() = default;
    void SetUp() {};
    void TearDown() {};
};

TEST(GlobalTest, BVHIntersectTest)
{
    Vector3f v0{50, 49, 49};
    Vector3f v1{50, 49, 51};
    Vector3f v2{50, 51, 49};
    Triangle *triangle1 = new Triangle(v0, v1, v2);

    std::vector<Object*> objects;
    objects.emplace_back(triangle1);
    Vector3f origin{0, 0, 0};
    Vector3f dir{1, 1, 1};
    Ray ray = Ray(origin, dir);
    auto bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
    auto intersection = bvh->Intersect(ray);
    ASSERT_EQ(intersection.happened, true);
}

TEST(GlobalTest, BVHNotIntersectTest)
{
    Vector3f v0{50, 49, 49};
    Vector3f v1{50, 49, 51};
    Vector3f v2{50, 51, 49};
    Triangle *triangle1 = new Triangle(v0, v1, v2);

    std::vector<Object*> objects;
    objects.emplace_back(triangle1);
    Vector3f origin{0, 0, 0};
    Vector3f dir{1, 1, 0};
    Ray ray = Ray(origin, dir);
    auto bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
    auto intersection = bvh->Intersect(ray);
    ASSERT_EQ(intersection.happened, false);
}


TEST(GlobalTest, Bounds3TestIntersect)
{
    Vector3f p_min{1, 1, 1};
    Vector3f p_max{2, 2, 2};
    Bounds3 bounds3 = Bounds3(p_min, p_max);
    Vector3f origin{0, 0, 0};
    Vector3f dir{1, 1, 1};
    Ray ray = Ray(origin, dir);
    Vector3f inv_dir;
    std::array<int, 3> dirIsNeg;
    auto ret = bounds3.IntersectP(ray, inv_dir, dirIsNeg);
    ASSERT_EQ(ret, true);
}

TEST(GlobalTest, Bounds3TestNotIntersect)
{
    Vector3f p_min{1, 1, 1};
    Vector3f p_max{2, 2, 2};
    Bounds3 bounds3 = Bounds3(p_min, p_max);
    Vector3f origin{0, 0, 0};
    Vector3f dir{1, 1, 0};
    Ray ray = Ray(origin, dir);
    Vector3f inv_dir;
    std::array<int, 3> dirIsNeg;
    auto ret = bounds3.IntersectP(ray, inv_dir, dirIsNeg);
    ASSERT_EQ(ret, false);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
