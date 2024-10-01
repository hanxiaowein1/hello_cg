#include "CGAL/Simple_cartesian.h"
#include "CGAL/Polyhedron_3.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <fstream>
#include <cassert>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Point_3 Point;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

typedef Polyhedron::Vertex Vertex;
typedef Polyhedron::Vertex_iterator Vertex_iterator;
typedef Polyhedron::Halfedge_handle Halfedge_handle;
typedef Polyhedron::Edge_iterator Edge_iterator;
typedef Polyhedron::Facet_iterator Facet_iterator;
typedef Polyhedron::Halfedge_around_vertex_const_circulator HV_circulator;
typedef Polyhedron::Halfedge_around_facet_circulator HF_circulator;

void create_center_vertex(Polyhedron &P, Facet_iterator f)
{
    Vector vec(0.0f, 0.0f, 0.0f);
    std::size_t order = 0;
    HF_circulator h = f->facet_begin();
    do{
        vec = vec + (h->vertex()->point() - CGAL::ORIGIN);
        ++order;
    }while(++h != f->facet_begin());
    assert(order >= 3);
    Point center = CGAL::ORIGIN + (vec / static_cast<double>(order));
    Halfedge_handle new_center = P.create_center_vertex(f->halfedge());
    // still unfinished
}

int main()
{
    // TODO:
    return 0;
}

