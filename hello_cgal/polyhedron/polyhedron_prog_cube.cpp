#include <iostream>

#include "CGAL/Simple_cartesian.h"
#include "CGAL/Polyhedron_3.h"

template <class Poly>
typename Poly::Halfedge_handle make_cube_3(Poly& P)
{
    CGAL_precondition(P.is_valid());
    typedef typename Poly::Point_3 Point;
    typedef typename Poly::Halfedge_handle Halfedge_handle;
    Halfedge_handle h = P.make_tetrahedron(
        Point(1, 0, 0),
        Point(0, 0, 1),
        Point(0, 0, 0),
        Point(0, 1, 0)
    );
    Halfedge_handle g = h->next()->opposite()->next();
    P.split_edge(h->next());
    P.split_edge(g->next());
    P.split_edge(g);
    h->next()->vertex()->point() = Point(1, 0, 1);
    g->next()->vertex()->point() = Point(0, 1, 1);
    g->opposite()->vertex()->point() = Point(1, 1, 0);
    Halfedge_handle f = P.split_facet(g->next(), g->next()->next()->next());
    Halfedge_handle e = P.split_edge(f);
    e->vertex()->point() = Point(1, 1, 1);
    P.split_facet(e, f->next()->next());
    CGAL_postcondition(P.is_valid());
    return h;
}

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::Halfedge_handle Halfedge_handle;

int main()
{
    Polyhedron P;
    Halfedge_handle h = make_cube_3(P);
    return (P.is_tetrahedron(h) ? 1 : 0);
}