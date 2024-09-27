#include <iostream>
#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"
#include "CGAL/convex_hull_2.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;

void array_convex_hull_2()
{
    Point_2 points[5] = {
        Point_2(0, 0),
        Point_2(10, 0),
        Point_2(10, 10),
        Point_2(6, 5),
        Point_2(4, 1)
    };
    Point_2 result[5];

    Point_2 *ptr = CGAL::convex_hull_2(points, points+5, result);
    std::cout << ptr - result << "points on the convex hull:" << std::endl;
    for(int i = 0; i < ptr - result; i++)
    {
        std::cout << result[i] << std::endl;
    }
}

class ExampleA
{
public:
    void print(){
        std::cout << "class ExampleA" << std::endl;
    }
};

class ExampleB
{
public:
    void print(){
        std::cout << "class ExampleB" << std::endl;
    }
};

template<typename T>
void personal_print(T ExampleClass)
{
    ExampleClass.print();
}

int main()
{
    ExampleA example_a;
    personal_print(example_a);
    return 0;
}