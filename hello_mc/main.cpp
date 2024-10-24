/*
  Vertices:            Edges:               Faces:
    3 ___________2        _____2______         ____________
   /|           /|      /|           /|      /|           /|
  / |          / |     B |          A |     / |    2     / |
7/___________6/  |    /_____6_____ /  |    /___________ /  |
|   |        |   |   |   3        |   1   |   |     4  |   |
|   |        |   |   |   |        |   |   | 3 |        | 1 |     z
|   0________|___1   |   |_____0__|___|   |   |_5______|___|     |
|  /         |  /    7  /         5  /    |  /         |  /      |____y
| /          | /     | 8          | 9     | /      0   | /      /
4/___________5/      |/_____4_____|/      |/___________|/      x

*/
#include <iostream>
#include <vector>
#include <unordered_map>
#include <bitset>
#include <format>
#include <string>
#include <fstream>
#include "Eigen/Dense"

enum class Vertex {v0, v1, v2, v3, v4, v5, v6, v7};

enum class Edge {e0, e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11};

enum class Face {f0, f1, f2, f3, f4, f5};

std::unordered_map<std::bitset<8>, std::vector<unsigned short int>> table = {
    {0b11111111, {}},

    /**
     * @brief case 1
     * 
     */
    {0b11111110, {0x038}},
    {0b11111101, {0x019}},
    {0b11111011, {0x12A}},
    {0b11110111, {0x23B}},
    {0b11101111, {0x478}},
    {0b11011111, {0x459}},
    {0b10111111, {0x56A}},
    {0b01111111, {0x67B}},

    /**
     * @brief case 2
     * 
     */
    {0b11111100, {0x381, 0x189}},
    {0b11111001, {0x2A0, 0x09A}},
    {0b11110011, {0x3BA, 0x13A}},
    {0b11100111, {0x2B0, 0xB80}},
    {0b11001111, {0x785, 0x589}},
    {0b10011111, {0x6A9, 0x649}},
    {0b00111111, {0x5AB, 0x5B7}},
    {0b01101111, {0x6B8, 0x684}},
    {0b11101110, {0x037, 0x074}},
    {0b11011101, {0x014, 0x145}},
    {0b10111011, {0x126, 0x165}},
    {0b01110111, {0x236, 0x367}},
};

Eigen::Vector3d get_vertex(
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance,
    bool on_x, bool on_y, bool on_z,
    Vertex first_point, Vertex second_point
);
void write_obj(std::string filename, const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& triangles);

int main()
{
    // std::vector<std::vector<std::vector<int>>> signed_distance{
    //     {
    //         {-1, 1},{1, 1}
    //     },
    //     {
    //         {1, 1},{1, 1}
    //     }
    // };
    // for(int k = 0; k < signed_distance.size(); k++)
    // {
    //     for(int j = 0; j < signed_distance[k].size(); j++)
    //     {
    //         for(int i = 0; i < signed_distance[k][j].size(); i++)
    //         {
    //             std::cout << signed_distance[k][j][i] << " ";
    //         }
    //     }
    // }
    // v0 ~ v7 signed distance
    std::vector<double> signed_distance{-1, -1, 1, 1, 1, 1, 1, 1};
    std::vector<Eigen::Vector3d> coors{
        {0,0,0},
        {0,1,0},
        {0,1,1},
        {0,0,1},
        {1,0,0},
        {1,1,0},
        {1,1,1},
        {1,0,1},
    };
    std::bitset<8> distance_symbol;
    int iso_value = 0;
    for(int i = 0; i < signed_distance.size(); i++)
    {
        if(signed_distance[i] - iso_value < 0)
        {
            distance_symbol.set(i, false);
        }
        else
        {
            distance_symbol.set(i, true);
        }
    }
    auto edgess = table.at(distance_symbol);
    
    // for(const auto& elem: edge)
    // {
    //     std::cout << std::format("{:#x} ", elem);
    // }
    // auto edge = table.find(distance_symbol);
    // if(edge != table.end())
    // {
    //     // std::cout << *edge << std::endl;
    //     for(const auto& elem: *edge)
    //     {
    //         std::cout << elem << " ";
    //     }
    // }
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    for(auto edges: edgess)
    {
        Eigen::Vector3i triangle;
        int triangle_edge_count = 0;
        while(triangle_edge_count <= 2)
        {
            auto edge = static_cast<Edge>(edges & 0xF);
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
            default:
                break;
            }
            vertices.emplace_back(std::move(vertex));
            triangle[triangle_edge_count] = vertices.size() - 1;
            triangle_edge_count++;
            edges = edges >> 4;
        }
        triangles.emplace_back(std::move(triangle));
    }
    write_obj("./default.obj", vertices, triangles);
    return 0;
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

Eigen::Vector3d get_vertex(
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance,
    bool on_x, bool on_y, bool on_z,
    Vertex first_point, Vertex second_point
)
{
    Eigen::Vector3d vertex;
    double t = signed_distance[static_cast<int>(first_point)] / (signed_distance[static_cast<int>(first_point)] - signed_distance[static_cast<int>(second_point)]);
    if(on_x)
    {
        vertex[0] = coors[static_cast<int>(first_point)].x() + t * std::abs(coors[static_cast<int>(first_point)].x() - coors[static_cast<int>(second_point)].x());
    }
    else
    {
        vertex[0] = coors[static_cast<int>(first_point)].x();
    }
    if(on_y)
    {
        vertex[1] = coors[static_cast<int>(first_point)].y() + t * std::abs(coors[static_cast<int>(first_point)].y() - coors[static_cast<int>(second_point)].y());
    }
    else
    {
        vertex[1] = coors[static_cast<int>(first_point)].y();
    }
    if(on_z)
    {
        vertex[2] = coors[static_cast<int>(first_point)].z() + t * std::abs(coors[static_cast<int>(first_point)].z() - coors[static_cast<int>(second_point)].z());
    }
    else
    {
        vertex[2] = coors[static_cast<int>(first_point)].z();
    }
    return vertex;
}