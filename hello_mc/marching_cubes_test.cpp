#include <fstream>
#include <stdexcept>
#include <format>
#include "gtest/gtest.h"
#include "mc_lookup_tables.h"
#include "marching_cubes.h"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "igl/signed_distance.h"
#include "igl/opengl/glfw/Viewer.h"
#include "igl/copyleft/cgal/points_inside_component.h"
#include "igl/is_vertex_manifold.h"

class GlobalTest
{
    ~GlobalTest() = default;
    void SetUp() {};
    void TearDown() {};
};

void write_obj(std::string filename, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    for(int i = 0; i < V.rows(); i++)
    {
        auto row = V.row(i);
        if(row.size() != 3)
        {
            throw std::invalid_argument(std::format("vertices col size is not equal to 3 at row {}", i));
        }
        file << "v " << row(0) << " " << row(1) << " " << row(2) << std::endl;
    }
    for(int i = 0; i < F.rows(); i++)
    {
        auto row = F.row(i);
        if(row.size() != 3)
        {
            throw std::invalid_argument(std::format("triangles col size is not equal to 3 at row {}", i));
        }
        file << "f " << row(0) + 1 << " " << row(1) + 1 << " " << row(2) + 1 << std::endl;
    }
	file.close();
}

bool inside_manifold_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::RowVector3d& point)
{
    Eigen::VectorXi inside;
    igl::copyleft::cgal::points_inside_component(V, F, point, inside);
    if (inside(0) == 1) {
        return true;
    }
    else {
        return false;
    }
}

TEST(GlobalTest, RotateMarchingCubes)
{
    init_tables();
    //invert_tables();
    print_mc_tables();
}

TEST(GlobalTest, CubeSignedDistance)
{
    init_tables();
    invert_tables();
    Eigen::MatrixXd V(8, 3);
    V << 0.5f, 0.5f, 0.5f,
         0.5f, 2.5f, 0.5f,
         0.5f, 2.5f, 2.5f,
         0.5f, 0.5f, 2.5f,
         2.5f, 0.5f, 0.5f,
         2.5f, 2.5f, 0.5f,
         2.5f, 2.5f, 2.5f,
         2.5f, 0.5f, 2.5f;

    int nx = 4, ny = 4, nz = 4;
    Eigen::MatrixXd P;
    P.resize(nx * ny * nz, 3);
    for(int k = 0; k < nz; k++)
    {
        for(int j = 0; j < ny; j++)
        {
            for(int i = 0; i < nx; i++)
            {
				unsigned int count = k * ny * nx + j * nx + i;
                P(count, 0) = i;
                P(count, 1) = j;
                P(count, 2) = k;
            }
        }
    }
    std::vector<double> S{
        0.866025f, 0.707107f, 0.707107f, 0.866025f,
        0.707107f, 0.5f, 0.5f, 0.707107f,
        0.707107f, 0.5f, 0.5f, 0.707107f,
        0.866025f, 0.707107f, 0.707107f, 0.866025f,

        0.707107f, 0.5f, 0.5f, 0.707107f,
        0.5f, -0.5f, -0.5f, 0.5f,
        0.5f, -0.5f, -0.5f, 0.5f,
        0.707107f, 0.5f, 0.5f, 0.707107f,

        0.707107f, 0.5f, 0.5f, 0.707107f,
        0.5f, -0.5f, -0.5f, 0.5f,
        0.5f, -0.5f, -0.5f, 0.5f,
        0.707107f, 0.5f, 0.5f, 0.707107f,

        0.866025f, 0.707107f, 0.707107f, 0.866025f,
        0.707107f, 0.5f, 0.5f, 0.707107f,
        0.707107f, 0.5f, 0.5f, 0.707107f,
        0.866025f, 0.707107f, 0.707107f, 0.866025f,
    };
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
                std::vector<unsigned short int> edgess;
                try
                {
                    edgess = MC_TABLES.at(distance_symbol);
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
    write_obj("./cube.obj", vertices, triangles);
}

/**
 * @brief test case 1
 * 
 */
TEST(GlobalTest, SphereSignedDistance)
{
    int nz = 3, ny = 3, nx = 3;
    std::vector<std::vector<std::vector<double>>> signed_distance(nz, std::vector<std::vector<double>>(ny, std::vector<double>(nx, 0.0f)));

    // initialize signed distance of sphere
    double radius = 0.8f;
	for(unsigned int k = 0; k < nz; k++)
	{
		for(unsigned int j = 0; j < ny; j++)
		{
			for(unsigned int i = 0; i < nx; i++)
			{
				auto square_x = std::pow(float(i) - (nx - 1) / 2, 2);
				auto square_y = std::pow(float(j) - (ny - 1) / 2, 2);
				auto square_z = std::pow(float(k) - (nz - 1) / 2, 2);
				signed_distance[k][j][i] = std::sqrt(square_x + square_y + square_z) - radius;
			}
		}
	}

    int iso_value = 0;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    // handle eaching marching cubes
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
                std::vector<double> cube_signed_distance{
                    signed_distance[i][j][k],
                    signed_distance[i][j + 1][k],
                    signed_distance[i][j + 1][k + 1],
                    signed_distance[i][j][k + 1],
                    signed_distance[i + 1][j][k],
                    signed_distance[i + 1][j + 1][k],
                    signed_distance[i + 1][j + 1][k + 1],
                    signed_distance[i + 1][j][k + 1],
                };
                std::bitset<8> distance_symbol;
                for(auto distance = cube_signed_distance.begin(); distance != cube_signed_distance.end(); distance++)
                {
                    int index = std::distance(cube_signed_distance.begin(), distance);
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
                    edgess = MC_TABLES.at(distance_symbol);
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
                        auto vertex = get_vertex_by_edge(edge, coors, cube_signed_distance);
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
    write_obj("./sphere.obj", vertices, triangles);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::GTEST_FLAG(filter) = "GlobalTest.CubeSignedDistance";
    int result = RUN_ALL_TESTS();
    return result;
}