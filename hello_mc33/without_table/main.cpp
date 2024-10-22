#include <iostream>
#include "mc33.h"
int main()
{
    std::vector<float> distances{1.0f, 2.0f, 2.0f, 3.0f, -1.0f, 1.0f, 2.0f, 1.0f};
    std::vector<Point3<float>> points{
        {0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f},
        {1.0f, 1.0f, 0.0f},
        {1.0f, 1.0f, 1.0f},
        {1.0f, 0.0f, 1.0f},
    };
    Cube cube = Cube(distances, points);
    // get negative points
    auto negative_points = cube.get_negative_points();
    // get connected points
    
    return 0;
}