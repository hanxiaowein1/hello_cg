#include "mc_lookup_tables.h"
#include <iostream>
#include <format>

// key: v8v7v6v5v4v3v2v1v0(0 is negative, 1 is positive, remember: bitset stores reversely)
std::unordered_map<std::bitset<8>, std::vector<unsigned short int>> MC_TABLES = {
     {0b11111111, {}},

    /**
     * @brief case 1
     * 
     */
    {0b11111110, {0x038}},  // v0
    // {0b11111101, {0x019}},  // v1
    // {0b11111011, {0x12A}},  // v2
    // {0b11110111, {0x23B}},  // v3
    // {0b11101111, {0x478}},  // v4
    // {0b11011111, {0x459}},  // v5
    // {0b10111111, {0x56A}},  // v6
    // {0b01111111, {0x67B}},  // v7

    /**
     * @brief case 2
     * 
     */
     {0b11111100, {0x381, 0x189}},  // v0 v1
    // {0b11111001, {0x2A0, 0x09A}},
    // {0b11110011, {0x3BA, 0x13A}},
    // {0b11110110, {0x2B0, 0xB80}},
    // {0b11001111, {0x785, 0x589}},
    // {0b10011111, {0x6A9, 0x649}},
    // {0b00111111, {0x5AB, 0x5B7}},
    // {0b01101111, {0x6B8, 0x684}},
    // {0b11101110, {0x037, 0x074}},
    // {0b11011101, {0x014, 0x145}},
    // {0b10111011, {0x126, 0x165}},
    // {0b01110111, {0x236, 0x367}},

    /**
     * @brief case 3
     * 
     */
     {0b11111010, {0x038, 0x12A}},  // v0 v2
    // {0b11110101, {0x23B, 0x019}},  // v1 v3
    // {0b11011110, {0x038, 0x459}},  // v0 v5
    // {0b11101101, {0x019, 0x478}},  // v1 v4
    // {0b10101111, {0x478, 0x56A}},  // v4 v6
    // {0b01011111, {0x459, 0x67B}},  // v5 v7
    // {0b01111011, {0x12A, 0x67B}},  // v2 v7
    // {0b10110111, {0x23B, 0x56A}},  // v3 v6
    // {0b10111101, {0x019, 0x56A}},  // v1 v6
    // {0b11011011, {0x12A, 0x459}},  // v2 v5
    // {0b01111110, {0x038, 0x67B}},  // v0 v7
    // {0b11100111, {0x23B, 0x478}},  // v3 v4

    /**
     * @brief case 4
     * 
     */
     {0b01111101, {0x019, 0x67B}},  // v1 v7
    // {0b11101011, {0x12A, 0x478}},  // v2 v4
    // {0b10111110, {0x038, 0x56A}},  // v0 v6
    // {0b11010111, {0x23B, 0x459}},  // v3 v5

    /**
     * @brief case 5
     * 
     */
    // face 0
    {0b11011100, {0x315, 0x358, 0x584}},  // v5 v0 v1
    // {0b11011100, {0x315, 0x354, 0x384}},  // v0 v1 v5
    // {0b11001101, {0x157, 0x178, 0x180}},  // v1 v5 v4
    // {0b11111111, {0x, 0x, 0x}},  // 

    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},

    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},

    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},

    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},
    // {0b11111111, {0x, 0x, 0x}},

    /**
     * @brief case 6
     * 
     */
    {0b11001011, {0x12A, 0x785, 0x598}},

    /**
     * @brief case 7
     * 
     */
    {0b01011011, {0x12A, 0x67B, 0x459}},

    /**
     * @brief case 8
     * 
     */
     {0b11001100, {0x137, 0x175}},  // face 0
    // {0b00110011, {0x137, 0x175}},  // face 2
    // {0b10011001, {0x026, 0x046}},  // face 1
    // {0b01100110, {0x026, 0x046}},  // face 3
    // {0b11110000, {0x9AB, 0x9B8}},  // face 4
    // {0b00001111, {0x9AB, 0x9B8}},  // face 5

    /**
     * @brief case 9
     * 
     */
    {0b11100100, {0x2B7, 0x274, 0x241, 0x149}},

    /**
     * @brief case 10
     * 
     */
    {0b01101001, {0x2A0, 0x0A9, 0x6B8, 0x684}},

    /**
     * @brief case 11
     * 
     */
    {0b11101000, {0x2A3, 0xA34, 0x374, 0xA94}},

    /**
     * @brief case 12
     * 
     */
    {0b01011100, {0x67B, 0x315, 0x358, 0x845}},

    /**
     * @brief case 13
     * 
     */
    {0b10100101, {0x019, 0x23B, 0x478, 0x56A}},

    /**
     * @brief case 14
     * 
     */
    {0b11010100, {0x48B, 0x4B1, 0x451, 0xB12}},
};

// clock wise
std::unordered_map<Vertex, Vertex> VERTEX_X_ROTATE = {
    {Vertex::v0, Vertex::v3},
    {Vertex::v1, Vertex::v0},
    {Vertex::v2, Vertex::v1},
    {Vertex::v3, Vertex::v2},
    {Vertex::v4, Vertex::v7},
    {Vertex::v5, Vertex::v4},
    {Vertex::v6, Vertex::v5},
    {Vertex::v7, Vertex::v6},
};

// clock wise
std::unordered_map<Vertex, Vertex> VERTEX_Y_ROTATE = {
    {Vertex::v0, Vertex::v4},
    {Vertex::v1, Vertex::v5},
    {Vertex::v2, Vertex::v1},
    {Vertex::v3, Vertex::v0},
    {Vertex::v4, Vertex::v7},
    {Vertex::v5, Vertex::v6},
    {Vertex::v6, Vertex::v2},
    {Vertex::v7, Vertex::v3},
};

// clock wise
std::unordered_map<Vertex, Vertex> VERTEX_Z_ROTATE = {
    {Vertex::v0, Vertex::v1},
    {Vertex::v1, Vertex::v5},
    {Vertex::v2, Vertex::v6},
    {Vertex::v3, Vertex::v2},
    {Vertex::v4, Vertex::v0},
    {Vertex::v5, Vertex::v4},
    {Vertex::v6, Vertex::v7},
    {Vertex::v7, Vertex::v3},
};

std::unordered_map<std::unordered_set<Vertex>, Edge, boost::hash<std::unordered_set<Vertex>>> VERTEX_EDGE = {
    {{Vertex::v0, Vertex::v1}, Edge::e0},
    {{Vertex::v1, Vertex::v2}, Edge::e1},
    {{Vertex::v2, Vertex::v3}, Edge::e2},
    {{Vertex::v3, Vertex::v0}, Edge::e3},
    {{Vertex::v4, Vertex::v5}, Edge::e4},
    {{Vertex::v5, Vertex::v6}, Edge::e5},
    {{Vertex::v6, Vertex::v7}, Edge::e6},
    {{Vertex::v7, Vertex::v4}, Edge::e7},
    {{Vertex::v0, Vertex::v4}, Edge::e8},
    {{Vertex::v1, Vertex::v5}, Edge::e9},
    {{Vertex::v2, Vertex::v6}, Edge::e10},
    {{Vertex::v3, Vertex::v7}, Edge::e11},
};

std::unordered_map<Edge, std::unordered_set<Vertex>> EDGE_VERTEX = {
    {Edge::e0, {Vertex::v0, Vertex::v1}},
    {Edge::e1, {Vertex::v1, Vertex::v2}},
    {Edge::e2, {Vertex::v2, Vertex::v3}},
    {Edge::e3, {Vertex::v3, Vertex::v0}},
    {Edge::e4, {Vertex::v4, Vertex::v5}},
    {Edge::e5, {Vertex::v5, Vertex::v6}},
    {Edge::e6, {Vertex::v6, Vertex::v7}},
    {Edge::e7, {Vertex::v7, Vertex::v4}},
    {Edge::e8, {Vertex::v0, Vertex::v4}},
    {Edge::e9, {Vertex::v1, Vertex::v5}},
    {Edge::e10, {Vertex::v2, Vertex::v6}},
    {Edge::e11, {Vertex::v3, Vertex::v7}},
};

std::vector<Vertex> get_negative_vertices_from_signed_distance(const std::bitset<8>& signed_distance)
{
    std::vector<Vertex> vertices;
    for(int i = 0; i < signed_distance.size(); i++)
    {
        if(signed_distance[i] == false)
        {
            vertices.emplace_back(static_cast<Vertex>(i));
        }
    }
    return vertices;
}

std::vector<Vertex> rotate_vertices_in_axis_z(const std::vector<Vertex>& vertices, int times = 1)
{
    std::vector<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        for(int i = 0; i < times; i++)
        {
            rotated_vertex = VERTEX_Z_ROTATE.at(rotated_vertex);
        }
        rotated_vertices.emplace_back(rotated_vertex);
    }
    return rotated_vertices;
}

std::vector<Vertex> rotate_vertices_in_axis_y(const std::vector<Vertex>& vertices, int times = 1)
{
    std::vector<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        for(int i = 0; i < times; i++)
        {
            rotated_vertex = VERTEX_Y_ROTATE.at(rotated_vertex);
        }
        rotated_vertices.emplace_back(rotated_vertex);
    }
    return rotated_vertices;
}

std::vector<Vertex> rotate_vertices_in_axis_x(const std::vector<Vertex>& vertices, int times = 1)
{
    std::vector<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        for(int i = 0; i < times; i++)
        {
            rotated_vertex = VERTEX_X_ROTATE.at(rotated_vertex);
        }
        rotated_vertices.emplace_back(rotated_vertex);
    }
    return rotated_vertices;
}

Edge rotate_edge_in_axis_z(const Edge& edge, int times = 1)
{
    auto vertices = EDGE_VERTEX.at(edge);
    std::unordered_set<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        for(int i = 0; i < times; i++)
        {
            rotated_vertex = VERTEX_Z_ROTATE.at(rotated_vertex);
        }
        rotated_vertices.emplace(rotated_vertex);
    }
    auto rotated_edge = VERTEX_EDGE.at(rotated_vertices);
    return rotated_edge;
}

Edge rotate_edge_in_axis_y(const Edge& edge, int times = 1)
{
    auto vertices = EDGE_VERTEX.at(edge);
    std::unordered_set<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        for(int i = 0; i < times; i++)
        {
            rotated_vertex = VERTEX_Y_ROTATE.at(rotated_vertex);
        }
        rotated_vertices.emplace(rotated_vertex);
    }
    auto rotated_edge = VERTEX_EDGE.at(rotated_vertices);
    return rotated_edge;
}

Edge rotate_edge_in_axis_x(const Edge& edge, int times = 1)
{
    auto vertices = EDGE_VERTEX.at(edge);
    std::unordered_set<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        for(int i = 0; i < times; i++)
        {
            rotated_vertex = VERTEX_X_ROTATE.at(rotated_vertex);
        }
        rotated_vertices.emplace(rotated_vertex);
    }
    auto rotated_edge = VERTEX_EDGE.at(rotated_vertices);
    return rotated_edge;
}

std::vector<unsigned short> rotate_edge_in_axis_z(const std::vector<unsigned short>& edgess, int times = 1)
{
    std::vector<unsigned short> rotated_edgess;
    for(auto edges: edgess)
    {
        unsigned short rotated_edges = 0x0000;
        for(int i = 0; i < 3; i++)
        {
            edges = edges >> (i == 0 ? 0 : 4);
            auto edge = edges & 0xF;
            auto rotated_edge = rotate_edge_in_axis_z(static_cast<Edge>(edge), times);
            rotated_edges = (rotated_edges | (static_cast<unsigned short>(rotated_edge) << (4 * i)));
        }
        rotated_edgess.emplace_back(rotated_edges);
    }
    return rotated_edgess;
}

std::vector<unsigned short> rotate_edge_in_axis_y(const std::vector<unsigned short>& edgess, int times = 1)
{
    std::vector<unsigned short> rotated_edgess;
    for(auto edges: edgess)
    {
        unsigned short rotated_edges = 0x0000;
        for(int i = 0; i < 3; i++)
        {
            edges = edges >> (i == 0 ? 0 : 4);
            auto edge = edges & 0xF;
            auto rotated_edge = rotate_edge_in_axis_y(static_cast<Edge>(edge), times);
            rotated_edges = (rotated_edges | (static_cast<unsigned short>(rotated_edge) << (4 * i)));
        }
        rotated_edgess.emplace_back(rotated_edges);
    }
    return rotated_edgess;
}

std::vector<unsigned short> rotate_edge_in_axis_x(const std::vector<unsigned short>& edgess, int times = 1)
{
    std::vector<unsigned short> rotated_edgess;
    for(auto edges: edgess)
    {
        unsigned short rotated_edges = 0x0000;
        for(int i = 0; i < 3; i++)
        {
            edges = edges >> (i == 0 ? 0 : 4);
            auto edge = edges & 0xF;
            auto rotated_edge = rotate_edge_in_axis_x(static_cast<Edge>(edge), times);
            rotated_edges = (rotated_edges | (static_cast<unsigned short>(rotated_edge) << (4 * i)));
        }
        rotated_edgess.emplace_back(rotated_edges);
    }
    return rotated_edgess;
}

void init_tables()
{
    std::unordered_map<std::bitset<8>, std::vector<unsigned short>> rotate_tables;
    for(auto [signed_distances, triangles]: MC_TABLES)
    {
        // get negative vertex
        auto vertices = get_negative_vertices_from_signed_distance(signed_distances);
        for(unsigned short k = 0; k < 4; k++)
        {
            // rotate in axis z
            auto z_rotated_vertices = rotate_vertices_in_axis_z(vertices, k);
            auto z_rotated_edgess = rotate_edge_in_axis_z(triangles, k);
            for(unsigned short j = 0; j < 4; j++)
            {
                // rotate in axis y
                auto y_rotated_vertices = rotate_vertices_in_axis_y(z_rotated_vertices, j);
                auto y_rotated_edgess = rotate_edge_in_axis_y(z_rotated_edgess, j);
                for(unsigned short i = 0; i < 4; i++)
                {
                    // rotate in axis x
                    auto x_rotated_vertices = rotate_vertices_in_axis_x(y_rotated_vertices, i);
                    auto x_rotated_edgess = rotate_edge_in_axis_x(y_rotated_edgess, i);
                    // use rotated vertices to construct new signed distance
                    std::bitset<8> rotated_signed_distance{0b11111111};
                    for(const auto& x_rotated_vertice: x_rotated_vertices)
                    {
                        rotated_signed_distance.set(static_cast<int>(x_rotated_vertice), false);
                    }
                    // then const new case
                    if(!rotate_tables.contains(rotated_signed_distance))
                    {
                        rotate_tables.emplace(std::make_pair(rotated_signed_distance, x_rotated_edgess));
                    }
                }
            }
        }
    }
    MC_TABLES.insert(rotate_tables.begin(), rotate_tables.end());
}

void invert_tables()
{
    std::unordered_map<std::bitset<8>, std::vector<unsigned short int>> invert_table;
    for(auto [key, value]: MC_TABLES)
    {
        auto origin = key;
        origin.flip();
        invert_table.emplace(std::make_pair(origin, value));
    }
    MC_TABLES.insert(invert_table.begin(), invert_table.end());
}

void print_mc_tables()
{
    for(const auto& [signed_distance, triangles]: MC_TABLES)
    {
        std::cout << signed_distance << ": ";
        for(const auto& triangle: triangles)
        {
            std::cout << std::format("{:#x} ", triangle) << " ";
        }
        std::cout << std::endl;
    }
}