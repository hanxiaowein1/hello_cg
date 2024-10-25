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
#include "mc_lookup_tables.h"
#include "marching_cubes.h"

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
    auto edgess = MC_Tables.at(distance_symbol);
    
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