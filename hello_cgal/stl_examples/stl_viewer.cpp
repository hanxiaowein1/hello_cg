#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;

int main() {
    Mesh mesh;
    if (!CGAL::IO::read_polygon_mesh("D:\\PHD\\Data\\Thingi10K\\Thingi10K\\raw_meshes\\34785.stl", mesh)) {
        std::cerr << "Error: cannot read file." << std::endl;
        return 1;
    }
    auto vcm = mesh.add_property_map<Mesh::Vertex_index, CGAL::IO::Color>("v:color").first;
    auto ecm = mesh.add_property_map<Mesh::Edge_index, CGAL::IO::Color>("e:color").first;
    auto fcm = mesh.add_property_map<Mesh::Face_index>("f:color", CGAL::IO::white() /*default*/).first;
    CGAL_USE(fcm);
    CGAL::draw(mesh);

    return 0;
}