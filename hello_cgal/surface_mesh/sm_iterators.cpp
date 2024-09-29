#include <vector>

#include "CGAL/Simple_cartesian.h"
#include "CGAL/Surface_mesh.h"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;

int main()
{
    Mesh m;
    vertex_descriptor u = m.add_vertex(Kernel::Point_3(0, 1, 0));
    vertex_descriptor v = m.add_vertex(Kernel::Point_3(0, 0, 0));
    vertex_descriptor w = m.add_vertex(Kernel::Point_3(1, 0, 0));
    vertex_descriptor x = m.add_vertex(Kernel::Point_3(1, 1, 0));

    m.add_face(u, v, w, x);

    {
        std::cout << "all vertices" << std::endl;

        Mesh::Vertex_range::iterator vb, ve;

        Mesh::Vertex_range r = m.vertices();

        vb = r.begin();
        ve = r.end();
        vb = std::begin(r);
        ve = std::end(r);

        for(boost::tie(vb, ve) = m.vertices(); vb != ve; ++vb)
        {
            std::cout << *vb << " " << m.point(*vb) << std::endl;
        }
        std::cout << "-----------------------separator line-----------------------" << std::endl;
        for(vertex_descriptor vd : m.vertices())
        {
            std::cout << vd << std::endl;
        }
    }

    return 0;
}