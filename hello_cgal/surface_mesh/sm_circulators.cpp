#include "CGAL/Simple_cartesian.h"
#include "CGAL/Surface_mesh.h"

#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;

int main()
{
    Mesh m;
    // Add the points as vertices
    vertex_descriptor u = m.add_vertex(Kernel::Point_3(0,1,0));
    vertex_descriptor v = m.add_vertex(Kernel::Point_3(0,0,0));
    vertex_descriptor w = m.add_vertex(Kernel::Point_3(1,0,0));
    vertex_descriptor x = m.add_vertex(Kernel::Point_3(1,1,0));

    face_descriptor f = m.add_face(u, v, w, x);
    {
        std::cout << "vertices around vertex" << v << std::endl;
        CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
        for(boost::tie(vbegin, vend) = vertices_around_face(m.halfedge(f), m); vbegin != vend; ++vbegin)
        {
            std::cout << *vbegin << std::endl;
        }
    }
    std::cout << "---------------------separator line-------------------\n";
    for(vertex_descriptor vd : vertices_around_face(m.halfedge(f), m))
    {
        std::cout << vd << std::endl;
    }
    return 0;
}