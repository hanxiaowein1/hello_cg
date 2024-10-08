#include <iostream>

#include "CGAL/Simple_cartesian.h"
#include "CGAL/Polyhedron_3.h"
#include "CGAL/draw_polyhedron.h"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::Halfedge_handle Halfedge_handle;

int main()
{
    Polyhedron P;
    Halfedge_handle h = P.make_tetrahedron();
    if(P.is_tetrahedron(h))
    {
        std::cout << "P is a tetrahedron" << std::endl;
        CGAL::draw(P);
        return 0;
    }
    return -1;
}