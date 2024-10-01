#include <iostream>
#include <algorithm>

#include "CGAL/Simple_cartesian.h"
#include "CGAL/Polyhedron_3.h"

struct Plane_equation
{
    template <typename Facet>
    typename Facet::Plane_3 operator()(Facet& f)
    {
        typename Facet::Halfedge_handle h = f.halfedge();
        typedef typename Facet::Plane_3 Plane;
        return Plane(
            h->vertex()->point(),
            h->next()->vertex()->point(),
            h->next()->next()->vertex()->point()
        );
    }
};

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::Halfedge_handle Halfedge_handle;

int main()
{
    Point_3 p(1, 0, 0);
    Point_3 q( 0, 1, 0);
    Point_3 r( 0, 0, 1);
    Point_3 s( 0, 0, 0);
    Polyhedron P;
    Halfedge_handle h = P.make_tetrahedron(p, q, r, s);
    // h->facet()->plane
    std::transform(P.facets_begin(), P.facets_end(), P.planes_begin(), Plane_equation());
    CGAL::IO::set_pretty_mode(std::cout);
    std::copy(P.planes_begin(), P.planes_end(), std::ostream_iterator<Plane_3>(std::cout, "\n"));
    return 0;
}