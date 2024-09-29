#include "CGAL/Simple_cartesian.h"
#include "CGAL/Surface_mesh.h"
#include "CGAL/draw_surface_mesh.h"
#include <format>
#include <iostream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;

int main()
{
    Mesh m;
    vertex_descriptor u = m.add_vertex(Kernel::Point_3(0, 1, 0));
    vertex_descriptor v = m.add_vertex(Kernel::Point_3(0, 0, 0));
    vertex_descriptor w = m.add_vertex(Kernel::Point_3(1, 1, 0));
    vertex_descriptor x = m.add_vertex(Kernel::Point_3(1, 0, 0));

    m.add_face(u, v, w);
    face_descriptor f = m.add_face(u, v, x);
    if(f == Mesh::null_face())
    {
        std::cerr<<"The face could not be added because of an orientation error."<<std::endl;
        f = m.add_face(u, x, v);
        // assert(f != Mesh::null_face());
        if(f == Mesh::null_face())
        {
            std::cerr<<"The face still could not be added because of an orientation error."<<std::endl;
        }
    }
    CGAL::draw(m);
    return 0;
}