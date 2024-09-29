#include <string>

#include "CGAL/Simple_cartesian.h"
#include "CGAL/Surface_mesh.h"
#include "CGAL/boost/graph/generators.h"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;

int main()
{
    Mesh m;
    vertex_descriptor v0 = m.add_vertex(Kernel::Point_3(0, 2, 0));
    vertex_descriptor v1 = m.add_vertex(Kernel::Point_3(2, 2, 0));
    vertex_descriptor v2 = m.add_vertex(Kernel::Point_3(0, 0, 0));
    vertex_descriptor v3 = m.add_vertex(Kernel::Point_3(2, 0, 0));
    vertex_descriptor v4 = m.add_vertex(Kernel::Point_3(1, 1, 0));

    m.add_face(v3, v1, v4);
    m.add_face(v0, v4, v1);
    m.add_face(v0, v2, v4);
    m.add_face(v2, v3, v4);

    Mesh::Property_map<vertex_descriptor, std::string> name;
    bool created;
    std::tie(name, created) = m.add_property_map<vertex_descriptor, std::string>("v:name", "m1");
    // auto [name, created] = m.add_property_map<vertex_descriptor, std::string>("v:name", "m1");
    assert(created);
    name[v0] = "hello";
    name[v2] = "world";

    {
        // auto [name, created] = m.add_property_map<vertex_descriptor, std::string>("v:name", "");
        Mesh::Property_map<vertex_descriptor, std::string> name;
        bool created;
        std::tie(name, created) = m.add_property_map<vertex_descriptor, std::string>("v:name", "");
        assert(!created);
    }

    Mesh::Property_map<face_descriptor, std::string> gnus;
    bool found;
    std::tie(gnus, found) = m.property_map<face_descriptor, std::string>("v:gnus");
    // auto [gnus, found] = m.property_map<face_descriptor, std::string>("v:gnus");
    assert(!found);

    Mesh::Property_map<vertex_descriptor, Kernel::Point_3> location = m.points();
    for(vertex_descriptor vd : m.vertices())
    {
        std::cout <<name[vd] << " @ " << location[vd] << std::endl;
    }
    std::cout << "--------------------separator line-------------------" << std::endl;

    std::vector<std::string> props = m.properties<vertex_descriptor>();
    for(std::string p : props)
    {
        std::cout << p << std::endl;
    }
    m.remove_property_map(name);
    return 0;
}