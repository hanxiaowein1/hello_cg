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

#ifndef __CHARLES_MC33_LOOKUP_TABLE__
#define __CHARLES_MC33_LOOKUP_TABLE__

#include "charles_mc33_type.h"
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <boost/functional/hash.hpp>

extern std::unordered_map<
    std::bitset<8>,
    std::unordered_map<
        std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>,
        std::vector<unsigned short>,
        boost::hash<std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>>
    >
> MC33_TABLES;

extern std::unordered_map<std::unordered_set<Vertex>, Edge, boost::hash<std::unordered_set<Vertex>>> VERTEX_EDGE;

extern std::unordered_map<Edge, std::unordered_set<Vertex>> EDGE_VERTEX;

bool vertex_connected(const std::bitset<8>& distance_sign, const Vertex &vertex1, const Vertex &vertex2);

bool vertex_interpolation_connected(const std::vector<double>& signed_distance, const Vertex& vertex1, const Vertex& vertex2);

void init_tables();

bool equal(const unsigned short& edges1, const unsigned short& edges2);

#endif