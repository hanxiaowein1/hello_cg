#include <vector>
#include <iterator>
#include <numeric>
#include <fstream>
#include <filesystem>

#include "Eigen/Dense"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "igl/readOFF.h"
#include "igl/readOBJ.h"
#include "igl/signed_distance.h"

#include "mc33.h"
#include "mc33_lookup_table.h"
#include "mc33_global_test.h"
#include "signed_distances_iface.h"


double interpolation_formula(const double& coor1, const double& coor2, const double& signed_distance1, const double& signed_distance2)
{
    double interpolated_coor = coor1 + (coor2 - coor1) * std::abs(signed_distance1) / (std::abs(signed_distance1) + std::abs(signed_distance2));
    return interpolated_coor;
}

double interpolate_between_four_edges(
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance,
    const Axis& axis
)
{
    std::vector<Edge> edges;
    std::vector<double> edges_interpolation(4, 0.0f);
    double interpolation_axis = 0.0f;
    switch(axis)
    {
    case Axis::x:
        edges = std::vector<Edge>{Edge::e8, Edge::e9, Edge::e10, Edge::e11};
        break;
    case Axis::y:
        edges = std::vector<Edge>{Edge::e0, Edge::e2, Edge::e4, Edge::e6};
        break;
    case Axis::z:
        edges = std::vector<Edge>{Edge::e1, Edge::e3, Edge::e5, Edge::e7};
        break;
    default:
        throw std::invalid_argument("invalid argument!");
    }
    auto formula = [](const double& coor1, const double& coor2, const double& signed_distance1, const double& signed_distance2) -> double {
        double interpolated_coor = coor1 + (coor2 - coor1) * std::abs(signed_distance1) / (std::abs(signed_distance1) + std::abs(signed_distance2));
        return interpolated_coor;
    };
    for(auto iterator = edges.begin(); iterator != edges.end(); iterator++)
    {
        int index = std::distance(edges.begin(), iterator);
        auto edge = *iterator;
        // get vertex from edge
        auto vertices = EDGE_VERTEX.at(edge);
        auto first_point = *(vertices.begin());
        auto second_point = *(++vertices.begin());
        //double t = 0.5f;
        //if (signed_distance[static_cast<int>(first_point)] != signed_distance[static_cast<int>(second_point)])
        //{
        //    t = signed_distance[static_cast<int>(first_point)] / (signed_distance[static_cast<int>(first_point)] - signed_distance[static_cast<int>(second_point)]);
        //}
        auto first_point_index = static_cast<int>(first_point);
        auto first_point_distance = signed_distance[first_point_index];
        auto second_point_index = static_cast<int>(second_point);
        auto second_point_distance = signed_distance[second_point_index];

        switch(axis)
        {
            case Axis::x:
                //edges_interpolation[index] = coors[static_cast<int>(first_point)].x() + t * std::abs(coors[static_cast<int>(first_point)].x() - coors[static_cast<int>(second_point)].x());
                edges_interpolation[index] = formula(coors[first_point_index].x(), coors[second_point_index].x(), first_point_distance, second_point_distance);
                break;
            case Axis::y:
                //edges_interpolation[index] = coors[static_cast<int>(first_point)].y() + t * std::abs(coors[static_cast<int>(first_point)].y() - coors[static_cast<int>(second_point)].y());
                edges_interpolation[index] = formula(coors[first_point_index].y(), coors[second_point_index].y(), first_point_distance, second_point_distance);
                break;
            case Axis::z:
                //edges_interpolation[index] = coors[static_cast<int>(first_point)].z() + t * std::abs(coors[static_cast<int>(first_point)].z() - coors[static_cast<int>(second_point)].z());
                edges_interpolation[index] = formula(coors[first_point_index].z(), coors[second_point_index].z(), first_point_distance, second_point_distance);
                break;
            default:
                throw std::invalid_argument("invalid argument!");
        }
        // edges_interpolation[index] = coors[static_cast<int>(first_point)].x() + t * std::abs(coors[static_cast<int>(first_point)].x() - coors[static_cast<int>(second_point)].x());
    }
    interpolation_axis = std::accumulate(edges_interpolation.begin(), edges_interpolation.end(), 0.0f) / edges_interpolation.size();

    return interpolation_axis;
}

// get interpolation point by all points on cube
Eigen::Vector3d get_vertex(
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance
)
{
    // interpolate between x(edge 8 9 A B)
    double interpolation_x = interpolate_between_four_edges(coors, signed_distance, Axis::x);
    double interpolation_y = interpolate_between_four_edges(coors, signed_distance, Axis::y);
    double interpolation_z = interpolate_between_four_edges(coors, signed_distance, Axis::z);
    Eigen::Vector3d vertex;
    vertex[0] = interpolation_x;
    vertex[1] = interpolation_y;
    vertex[2] = interpolation_z;
    return vertex;
}

// interpolation between two points on same edge
Eigen::Vector3d get_vertex(
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance,
    bool on_x, bool on_y, bool on_z,
    Vertex first_point, Vertex second_point
)
{
    Eigen::Vector3d vertex;
    // first point index
    auto p1_i = static_cast<int>(first_point);
    // first point coordinate
    auto p1_c = coors[p1_i];
    // first point signed distance
    auto sd1 = signed_distance[p1_i];
    auto p2_i = static_cast<int>(second_point);
    auto p2_c = coors[p2_i];
    auto sd2 = signed_distance[p2_i];
    // double t = signed_distance[static_cast<int>(first_point)] / (signed_distance[static_cast<int>(first_point)] - signed_distance[static_cast<int>(second_point)]);
    if(on_x)
    {
        // vertex[0] = coors[static_cast<int>(first_point)].x() + t * std::abs(coors[static_cast<int>(first_point)].x() - coors[static_cast<int>(second_point)].x());
        vertex[0] = interpolation_formula(p1_c.x(), p2_c.x(), sd1, sd2);
    }
    else
    {
        // vertex[0] = coors[static_cast<int>(first_point)].x();
        vertex[0] = p1_c.x();
    }
    if(on_y)
    {
        // vertex[1] = coors[static_cast<int>(first_point)].y() + t * std::abs(coors[static_cast<int>(first_point)].y() - coors[static_cast<int>(second_point)].y());
        vertex[1] = interpolation_formula(p1_c.y(), p2_c.y(), sd1, sd2);
    }
    else
    {
        // vertex[1] = coors[static_cast<int>(first_point)].y();
        vertex[1] = p1_c.y();
    }
    if(on_z)
    {
        // vertex[2] = coors[static_cast<int>(first_point)].z() + t * std::abs(coors[static_cast<int>(first_point)].z() - coors[static_cast<int>(second_point)].z());
        vertex[2] = interpolation_formula(p1_c.z(), p2_c.z(), sd1, sd2);
    }
    else
    {
        // vertex[2] = coors[static_cast<int>(first_point)].z();
        vertex[2] = p1_c.z();
    }
    return vertex;
}

Eigen::Vector3d get_vertex_by_edge(
    Edge edge,
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance
)
{
    Eigen::Vector3d vertex;
    switch (edge)
    {
    case Edge::e0:
        // vertex 0 1
        vertex = get_vertex(coors, signed_distance, false, true, false, Vertex::v0, Vertex::v1);
        break;
    case Edge::e1:
        // vertex 1 2
        vertex = get_vertex(coors, signed_distance, false, false, true, Vertex::v1, Vertex::v2);
        break;
    case Edge::e2:
        // vertex 2 3
        vertex = get_vertex(coors, signed_distance, false, true, false, Vertex::v3, Vertex::v2);
        break;
    case Edge::e3:
        // vertex 0 3
        vertex = get_vertex(coors, signed_distance, false, false, true, Vertex::v0, Vertex::v3);
        break;
    case Edge::e4:
        // vertex 4 5
        vertex = get_vertex(coors, signed_distance, false, true, false, Vertex::v4, Vertex::v5);
        break;
    case Edge::e5:
        // vertex 5 6
        vertex = get_vertex(coors, signed_distance, false, false, true, Vertex::v5, Vertex::v6);
        break;
    case Edge::e6:
        // vertex 7 6
        vertex = get_vertex(coors, signed_distance, false, true, false, Vertex::v7, Vertex::v6);
        break;
    case Edge::e7:
        // vertex 4 7
        vertex = get_vertex(coors, signed_distance, false, false, true, Vertex::v4, Vertex::v7);
        break;
    case Edge::e8:
        // vertex 0 4
        vertex = get_vertex(coors, signed_distance, true, false, false, Vertex::v0, Vertex::v4);
        break;
    case Edge::e9:
        // vertex 1 5
        vertex = get_vertex(coors, signed_distance, true, false, false, Vertex::v1, Vertex::v5);
        break;
    case Edge::e10:
        // vertex 2 6
        vertex = get_vertex(coors, signed_distance, true, false, false, Vertex::v2, Vertex::v6);
        break;
    case Edge::e11:
        // vertex 3 7
        vertex = get_vertex(coors, signed_distance, true, false, false, Vertex::v3, Vertex::v7);
        break;
    case Edge::ec:
        // center interpolated point
        vertex = get_vertex(coors, signed_distance);
        break;
    default:
        throw std::invalid_argument("invalid edge!");
        // break;
    }
    return vertex;
}

std::vector<unsigned short> get_edgess(const std::vector<double>& signed_distances, const decltype(MC33_TABLES)& tables)
{
    std::bitset<8> distances_signs{0b11111111};
    for(int i = 0; i < signed_distances.size(); i++)
    {
        auto signed_distance = signed_distances[i];
        if(signed_distance < 0)
        {
            distances_signs.set(i, false);
        }
    }
    auto sub_cases = tables.at(distances_signs);
    // get all connected vertices that are not ambiguous
    std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>> connected_verticess;
    for(unsigned short i = 0; i < 8; i++)
    {
        for(unsigned short j = i + 1; j < 8; j++)
        {
            Vertex vertex1 = static_cast<Vertex>(i), vertex2 = static_cast<Vertex>(j);
            if (distances_signs[static_cast<unsigned short>(vertex1)] == distances_signs[static_cast<unsigned short>(vertex2)])
            {
                if (!vertex_connected(distances_signs, vertex1, vertex2))
                {
                    if (vertex_interpolation_connected(signed_distances, vertex1, vertex2))
                    {
                        connected_verticess.emplace(std::unordered_set<Vertex>{vertex1, vertex2});
                    }
                }
            }
        }
    }

    // get egess by connected vertices
    auto triangles = sub_cases.at(connected_verticess);
    return triangles;
}

void write_obj(std::string filename, const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& triangles)
{
	std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    for(const auto& vertice: vertices)
    {
        file << "v " << vertice[0] << " " << vertice[1] << " " << vertice[2] << std::endl;
    }
    for(const auto& triangle: triangles)
    {
		file << "f " << triangle[0] + 1 << " " << triangle[1] + 1 << " " << triangle[2] + 1 << std::endl;
    }
	file.close();
}

void generate_mesh(const int& nx, const int& ny, const int& nz, const Eigen::VectorXd& signed_distances, const std::string& save_path)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    int iso_value = 0;

    for(int k = 0; k < nz - 1; k++)
	{
		for(int j = 0; j < ny - 1; j++)
		{
			for(int i = 0; i < nx - 1; i++)
			{
				//  get cube
                std::vector<Eigen::Vector3d> coors{
                    {static_cast<double>(i), static_cast<double>(j), static_cast<double>(k)},
                    {static_cast<double>(i), static_cast<double>(j + 1), static_cast<double>(k)},
                    {static_cast<double>(i), static_cast<double>(j + 1), static_cast<double>(k + 1)},
                    {static_cast<double>(i), static_cast<double>(j), static_cast<double>(k + 1)},
                    {static_cast<double>(i + 1), static_cast<double>(j), static_cast<double>(k)},
                    {static_cast<double>(i + 1), static_cast<double>(j + 1), static_cast<double>(k)},
                    {static_cast<double>(i + 1), static_cast<double>(j + 1), static_cast<double>(k + 1)},
                    {static_cast<double>(i + 1), static_cast<double>(j), static_cast<double>(k + 1)},
                };
				unsigned int count = k * ny * nx + j * nx + i;
                std::vector<double> signed_distance{
                    signed_distances[k * ny * nx + j * nx + i],
                    signed_distances[k * ny * nx + (j + 1) * nx + i],
                    signed_distances[(k + 1) * ny * nx + (j + 1) * nx + i],
                    signed_distances[(k + 1) * ny * nx + j * nx + i],
                    signed_distances[k * ny * nx + j * nx + i + 1],
                    signed_distances[k * ny * nx + (j + 1) * nx + i + 1],
                    signed_distances[(k + 1) * ny * nx + (j + 1) * nx + i + 1],
                    signed_distances[(k + 1) * ny * nx + j * nx + i + 1],
                };

                std::bitset<8> distance_symbol;
                for(auto distance = signed_distance.begin(); distance != signed_distance.end(); distance++)
                {
                    int index = std::distance(signed_distance.begin(), distance);
                    if(*distance - iso_value < 0)
                    {
                        distance_symbol.set(index, false);
                    }
                    else
                    {
                        distance_symbol.set(index, true);
                    }
                }
                std::vector<unsigned short int> edgess;
                try
                {
                    // edgess = MC_TABLES.at(distance_symbol);
                    edgess = get_edgess(signed_distance, MC33_TABLES);
                }
                catch (const std::out_of_range& e)
                {
                    std::cerr << "Caught std::out_of_range exception: " << e.what() << std::endl;
                    throw e;
                }
                for(auto edges: edgess)
                {
                    Eigen::Vector3i triangle;
                    int triangle_edge_count = 0;
                    while(triangle_edge_count <= 2)
                    {
                        auto edge = static_cast<Edge>(edges & 0xF);
                        auto vertex = get_vertex_by_edge(edge, coors, signed_distance);
                        vertices.emplace_back(std::move(vertex));
                        triangle[triangle_edge_count] = vertices.size() - 1;
                        triangle_edge_count++;
                        edges = edges >> 4;
                    }
                    triangles.emplace_back(std::move(triangle));
                }
            }
        }
    }
    write_obj(save_path, vertices, triangles);
}

void generate_mesh(const std::string& sd_path, const std::string save_path)
{
    int nx = 0, ny = 0, nz = 0;
    auto signed_distances = get_signed_distance(nx, ny, nz, sd_path);
    generate_mesh(nx, ny, nz, signed_distances, save_path);
}

TEST(GlobalTest, generate_and_serialize_signed_distances)
{
    std::string pb_save_path = "./signed_distances.pb";
    std::string off_path = "D:\\Library\\libigl\\build\\_deps\\libigl_tutorial_tata-src\\bunny.off";
    generate_signed_distance(20, 20, 20, off_path, pb_save_path);
    ASSERT_EQ(true, std::filesystem::exists(pb_save_path));
    ASSERT_NE(0, std::filesystem::file_size(pb_save_path));
}

TEST(GlobalTest, deserialize_sf_and_generate_mesh)
{
    std::string sd_path = "./signed_distances.pb";
    std::string save_path = "./proto_bunny.obj";
    generate_mesh(sd_path, save_path);
    ASSERT_EQ(true, std::filesystem::exists(save_path));
    ASSERT_NE(0, std::filesystem::file_size(save_path));
}

TEST(GlobalTest, IGLBunny)
{
    init_tables();

    int nx = 20;
    int ny = 20;
    int nz = 20;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    std::string off_path = "D:\\Library\\libigl\\build\\_deps\\libigl_tutorial_tata-src\\bunny.off";
    std::string obj_path = "D:\\PHD\\Projects\\DevelopApp\\DevelopApp\\model\\Bunny.obj";
    // igl::readOFF(off_path, V, F);
    igl::readOBJ(obj_path, V, F);
    
    Eigen::Vector3d m = V.colwise().minCoeff();
    std::cout << m << std::endl;
    Eigen::Vector3d M = V.colwise().maxCoeff();
    std::cout << M << std::endl;
    Eigen::MatrixXd P;
	P.resize(nx * ny * nz, 3);
	for(unsigned int k = 0; k < nz; k++)
	{
		double z_axis = m.z() + double(k) * (M.z() - m.z()) / double(nz - 1);
		for(unsigned int j = 0; j < ny; j++)
		{
			double y_axis = m.y() + double(j) * (M.y() - m.y()) / double(ny - 1);
			for(unsigned int i = 0; i < nx; i++)
			{
				double x_axis = m.x() + double(i) * (M.x() - m.x()) / double(nx - 1);
				unsigned int count = k * ny * nx + j * nx + i;
				P(count, 0) = x_axis;
				P(count, 1) = y_axis;
				P(count, 2) = z_axis;
			}
		}
	}

    Eigen::VectorXi I;
    Eigen::MatrixXd N,C;
    Eigen::VectorXd S;
    igl::SignedDistanceType type = igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL;
    igl::signed_distance(P, V, F, type, S, I, C, N);

    // std::cout << "------------------S---------------------" << std::endl;
    // std::cout << S << std::endl;

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    int iso_value = 0;

    for(int k = 0; k < nz - 1; k++)
	{
		for(int j = 0; j < ny - 1; j++)
		{
			for(int i = 0; i < nx - 1; i++)
			{
				//  get cube
                std::vector<Eigen::Vector3d> coors{
                    {static_cast<double>(i), static_cast<double>(j), static_cast<double>(k)},
                    {static_cast<double>(i), static_cast<double>(j + 1), static_cast<double>(k)},
                    {static_cast<double>(i), static_cast<double>(j + 1), static_cast<double>(k + 1)},
                    {static_cast<double>(i), static_cast<double>(j), static_cast<double>(k + 1)},
                    {static_cast<double>(i + 1), static_cast<double>(j), static_cast<double>(k)},
                    {static_cast<double>(i + 1), static_cast<double>(j + 1), static_cast<double>(k)},
                    {static_cast<double>(i + 1), static_cast<double>(j + 1), static_cast<double>(k + 1)},
                    {static_cast<double>(i + 1), static_cast<double>(j), static_cast<double>(k + 1)},
                };
				unsigned int count = k * ny * nx + j * nx + i;
                std::vector<double> signed_distance{
                    S[k * ny * nx + j * nx + i],
                    S[k * ny * nx + (j + 1) * nx + i],
                    S[(k + 1) * ny * nx + (j + 1) * nx + i],
                    S[(k + 1) * ny * nx + j * nx + i],
                    S[k * ny * nx + j * nx + i + 1],
                    S[k * ny * nx + (j + 1) * nx + i + 1],
                    S[(k + 1) * ny * nx + (j + 1) * nx + i + 1],
                    S[(k + 1) * ny * nx + j * nx + i + 1],
                };

                std::bitset<8> distance_symbol;
                for(auto distance = signed_distance.begin(); distance != signed_distance.end(); distance++)
                {
                    int index = std::distance(signed_distance.begin(), distance);
                    if(*distance - iso_value < 0)
                    {
                        distance_symbol.set(index, false);
                    }
                    else
                    {
                        distance_symbol.set(index, true);
                    }
                }
                std::vector<unsigned short> edgess;
                try
                {
                    // edgess = MC_TABLES.at(distance_symbol);
                    edgess = get_edgess(signed_distance, MC33_TABLES);
                }
                catch (const std::out_of_range& e)
                {
                    std::cerr << "Caught std::out_of_range exception: " << e.what() << std::endl;
                    throw e;
                }
                for(auto edges: edgess)
                {
                    Eigen::Vector3i triangle;
                    int triangle_edge_count = 0;
                    unsigned short get_edge_factor = 0xF00;
                    while(triangle_edge_count <= 2)
                    {
                        auto edge = static_cast<Edge>(
                            (edges & (get_edge_factor >> (triangle_edge_count * 4)))
                            >>
                            (4 * (2 - triangle_edge_count))
                        );
                        auto vertex = get_vertex_by_edge(edge, coors, signed_distance);
                        vertices.emplace_back(std::move(vertex));
                        triangle[triangle_edge_count] = vertices.size() - 1;
                        triangle_edge_count++;
                        // edges = edges >> 4;
                    }
                    triangles.emplace_back(std::move(triangle));
                }
            }
        }
    }
    write_obj("./problematic_bunny.obj", vertices, triangles);
}

TEST(GlobalTest, interpolate_between_four_edges)
{
    std::vector<Eigen::Vector3d> coors = {
        {0, 0, 0},
        {0, 1, 0},
        {0, 1, 1},
        {0, 0, 1},
        {1, 0, 0},
        {1, 1, 0},
        {1, 1, 1},
        {1, 0, 1},
    };
    // simple case
    {
        std::vector<double> signed_distances = {
            -1, -1, -1, -1, 1, 1, 1, 1
        };
        Axis axis = Axis::x;
        auto value = interpolate_between_four_edges(
            coors, signed_distances, axis
        );
        ASSERT_EQ(value, 0.5f);
    }
    // axis in all directions {x, y, z}
    {
        // x axis
        {
            std::vector<double> signed_distances = {
                -2, -2, -2, -2, 1, 1, 1, 1
            };
            Axis axis = Axis::x;
            auto value = interpolate_between_four_edges(
                coors, signed_distances, axis
            );
            ASSERT_LT(std::abs(value - 0.666666f), 0.000001f);
        }
        // y axis
        {
            std::vector<double> signed_distances = {
                -2, 1, 1, -2, -2, 1, 1, -2
            };
            Axis axis = Axis::y;
            auto value = interpolate_between_four_edges(
                coors, signed_distances, axis
            );
            ASSERT_LT(std::abs(value - 0.666666f), 0.000001f);
        }
        // z axis
        {
            std::vector<double> signed_distances = {
                -2, -2, 1, 1, -2, -2, 1, 1
            };
            Axis axis = Axis::z;
            auto value = interpolate_between_four_edges(
                coors, signed_distances, axis
            );
            ASSERT_LT(std::abs(value - 0.666666f), 0.000001f);
        }
    }
    // some more complex version
    {
        std::vector<double> signed_distances = {
            -1, -3, -1, -1, 3, 5, 15, 7
        };
        {
            Axis axis = Axis::x;
            auto value = interpolate_between_four_edges(
                coors, signed_distances, axis
            );
            ASSERT_LT(std::abs(value - 13.0f / (16.0f * 4.0f)), 0.000001f);
        }
        {
            Axis axis = Axis::y;
            auto value = interpolate_between_four_edges(
                coors, signed_distances, axis
            );
            ASSERT_LT(std::abs(value - (9.0f / 8.0f + 7.0f / 22.0f) / 4.0f), 0.000001f);
        }
        {
            Axis axis = Axis::z;
            auto value = interpolate_between_four_edges(
                coors, signed_distances, axis
            );
            ASSERT_LT(std::abs(value - (9.0f / 5.0f) / 4.0f), 0.000001f);
        }
    }
}

TEST(GlobalTest, get_vertex_const_vector_Vector3d_const_vector_double)
{
    std::vector<Eigen::Vector3d> coors = {
        {0, 0, 0},
        {0, 1, 0},
        {0, 1, 1},
        {0, 0, 1},
        {1, 0, 0},
        {1, 1, 0},
        {1, 1, 1},
        {1, 0, 1},
    };
    std::vector<double> signed_distances = {
        -1, -3, -1, -1, 3, 5, 15, 7
    };
    auto value = get_vertex(coors, signed_distances);
    ASSERT_LT(std::abs(value.x() - 13.0f / (16.0f * 4.0f)), 0.000001f);
    ASSERT_LT(std::abs(value.y() - (9.0f / 8.0f + 7.0f / 22.0f) / 4.0f), 0.000001f);
    ASSERT_LT(std::abs(value.z() - (9.0f / 5.0f) / 4.0f), 0.000001f);
}

TEST(GlobalTest, get_vertex_const_vector_eigen_const_vector_double_bool_bool_bool_Vertex_Vertex)
{
    std::vector<Eigen::Vector3d> coors = {
        {0, 0, 0},
        {0, 1, 0},
        {0, 1, 1},
        {0, 0, 1},
        {1, 0, 0},
        {1, 1, 0},
        {1, 1, 1},
        {1, 0, 1},
    };
    std::vector<double> signed_distances = {
        -1, -3, -1, -1, 3, 5, 15, 7
    };
    // on axis x
    {
        {
            auto value = get_vertex(coors, signed_distances, true, false, false, Vertex::v0, Vertex::v4);
            ASSERT_LT(std::abs(value.x() - 0.25f), 0.000001f);
            ASSERT_EQ(value.y(), 0.0f);
            ASSERT_EQ(value.z(), 0.0f);
        }
        {
            auto value = get_vertex(coors, signed_distances, true, false, false, Vertex::v4, Vertex::v0);
            ASSERT_LT(std::abs(value.x() - 0.25f), 0.000001f);
            ASSERT_EQ(value.y(), 0.0f);
            ASSERT_EQ(value.z(), 0.0f);
        }
        {
            auto value = get_vertex(coors, signed_distances, true, false, false, Vertex::v1, Vertex::v5);
            ASSERT_LT(std::abs(value.x() - 3.0f / 8.0f), 0.000001f);
            ASSERT_EQ(value.y(), 1.0f);
            ASSERT_EQ(value.z(), 0.0f);
        }
    }
    // on axis y
    {
        auto value = get_vertex(coors, signed_distances, false, true, false, Vertex::v0, Vertex::v1);
        ASSERT_LT(std::abs(value.y() - 0.25f), 0.000001f);
        ASSERT_EQ(value.x(), 0.0f);
        ASSERT_EQ(value.z(), 0.0f);
    }
    // on axis z
    {
        auto value = get_vertex(coors, signed_distances, false, false, true, Vertex::v0, Vertex::v3);
        ASSERT_LT(std::abs(value.z() - 0.5f), 0.000001f);
        ASSERT_EQ(value.x(), 0.0f);
        ASSERT_EQ(value.y(), 0.0f);
    }
}

TEST(GlobalTest, get_vertex_by_edge_const_vector_Vector3d_const_vector_double)
{
    std::vector<Eigen::Vector3d> coors = {
        {1, 1, 1},
        {1, 2, 1},
        {1, 2, 2},
        {1, 1, 2},
        {2, 1, 1},
        {2, 2, 1},
        {2, 2, 2},
        {2, 1, 2},
    };
    std::vector<double> signed_distances = {
        -1, -3, -1, -1, 3, 5, 15, 7
    };
    {
        // e0
        auto vertex = get_vertex_by_edge(Edge::e0, coors, signed_distances);
        ASSERT_LT(std::abs(vertex.y() - 1.25f), 0.000001f);
        ASSERT_EQ(vertex.x(), 1.0f);
        ASSERT_EQ(vertex.z(), 1.0f);
    }
    {
        // e1
        auto vertex = get_vertex_by_edge(Edge::e1, coors, signed_distances);
        ASSERT_LT(std::abs(vertex.z() - 1.75f), 0.000001f);
        ASSERT_EQ(vertex.y(), 2.0f);
        ASSERT_EQ(vertex.x(), 1.0f);
    }
    {
        // e2
        auto vertex = get_vertex_by_edge(Edge::e2, coors, signed_distances);
        ASSERT_EQ(vertex.x(), 1.0f);
        ASSERT_LT(std::abs(vertex.y() - 1.5f), 0.000001f);
        ASSERT_EQ(vertex.z(), 2.0f);
    }
    {
        // e3
        auto vertex = get_vertex_by_edge(Edge::e3, coors, signed_distances);
        ASSERT_EQ(vertex.x(), 1.0f);
        ASSERT_EQ(vertex.y(), 1.0f);
        ASSERT_LT(std::abs(vertex.z() - 1.5f), 0.000001f);
    }
    {
        // e4
        auto vertex = get_vertex_by_edge(Edge::e4, coors, signed_distances);
        ASSERT_EQ(vertex.x(), 2.0f);
        ASSERT_LT(std::abs(vertex.y() - (1.0f + 3.0f / 8.0f)), 0.000001f);
        ASSERT_EQ(vertex.z(), 1.0f);
    }
    {
        // e5
        auto vertex = get_vertex_by_edge(Edge::e5, coors, signed_distances);
        ASSERT_EQ(vertex.x(), 2.0f);
        ASSERT_EQ(vertex.y(), 2.0f);
        ASSERT_LT(std::abs(vertex.z() - (1.0f + 5.0f / 20.0f)), 0.000001f);
    }
    {
        // e6
        auto vertex = get_vertex_by_edge(Edge::e6, coors, signed_distances);
        ASSERT_EQ(vertex.x(), 2.0f);
        ASSERT_LT(std::abs(vertex.y() - (1.0f + 7.0f / 22.0f)), 0.000001f);
        ASSERT_EQ(vertex.z(), 2.0f);
    }
    {
        // e7
        auto vertex = get_vertex_by_edge(Edge::e7, coors, signed_distances);
        ASSERT_EQ(vertex.x(), 2.0f);
        ASSERT_EQ(vertex.y(), 1.0f);
        ASSERT_LT(std::abs(vertex.z() - (1.0f + 3.0f / 10.0f)), 0.000001f);
    }
    {
        // e8
        auto vertex = get_vertex_by_edge(Edge::e8, coors, signed_distances);
        ASSERT_LT(std::abs(vertex.x() - (1.0f + 0.25f)), 0.000001f);
        ASSERT_EQ(vertex.y(), 1.0f);
        ASSERT_EQ(vertex.z(), 1.0f);
    }
    {
        // e9
        auto vertex = get_vertex_by_edge(Edge::e9, coors, signed_distances);
        ASSERT_LT(std::abs(vertex.x() - (1.0f + 3.0f / 8.0f)), 0.000001f);
        ASSERT_EQ(vertex.y(), 2.0f);
        ASSERT_EQ(vertex.z(), 1.0f);
    }
    {
        // e10
        auto vertex = get_vertex_by_edge(Edge::e10, coors, signed_distances);
        ASSERT_LT(std::abs(vertex.x() - (1.0f + 1.0f / 16.0f)), 0.000001f);
        ASSERT_EQ(vertex.y(), 2.0f);
        ASSERT_EQ(vertex.z(), 2.0f);
    }
    {
        // e11
        auto vertex = get_vertex_by_edge(Edge::e11, coors, signed_distances);
        ASSERT_LT(std::abs(vertex.x() - (1.0f + 1.0f / 8.0f)), 0.000001f);
        ASSERT_EQ(vertex.y(), 1.0f);
        ASSERT_EQ(vertex.z(), 2.0f);
    }
    {
        // ec
        auto vertex = get_vertex_by_edge(Edge::ec, coors, signed_distances);
        ASSERT_LT(std::abs(vertex.x() - (1.0f + 13.0f / (16.0f * 4.0f))), 0.000001f);
        ASSERT_LT(std::abs(vertex.y() - (1.0f + (9.0f / 8.0f + 7.0f / 22.0f) / 4.0f)), 0.000001f);
        ASSERT_LT(std::abs(vertex.z() - (1.0f + (9.0f / 5.0f) / 4.0f)), 0.000001f);
    }
}

TEST(GlobalTest, get_edgess_const_vector_double_const_unordered_map)
{
    decltype(MC33_TABLES) tables = {
        {
            0b10101111,
            {
                // case 3.1
                {
                    {{Vertex::v5, Vertex::v7}}, {0x487, 0x56A}
                },
                // case 3.2
                {
                    {{Vertex::v4, Vertex::v6}}, {0x876, 0x86A, 0x8A5, 0x854}
                },
            }
        },
    };
    std::vector<double> signed_distances = {
        1.0f, 1.0f, 1.0f, 1.0f, -2.0f, 1.0f, -2.0f, 1.0f
    };

    auto triangles = get_edgess(signed_distances, tables);
    decltype(triangles) expected_triangles = {0x876, 0x86A, 0x8A5, 0x854};
    ASSERT_EQ(triangles.size(), expected_triangles.size());
    for(int i = 0; i < triangles.size(); i++)
    {
        ASSERT_EQ(true, equal(triangles[i], expected_triangles[i])) << std::format("triangle is not the expected one at index {}, real output is {:#0x}, expected is {:#0x}", i, triangles[i], expected_triangles[i]);
    }
}