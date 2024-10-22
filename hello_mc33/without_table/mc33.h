#ifndef _CHARLES_MC33_H_
#define _CHARLES_MC33_H_

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <bitset>
#include "basic_type.h"
#include "cgl.h"

template<typename DistanceType>
class Cube
{
public :
    Cube() = delete;
    Cube(std::vector<DistanceType> &distances, std::vector<Point3<DistanceType>> &points);
    ~Cube() = default;
    std::vector<unsigned short> get_negative_points();
    std::unordered_set<std::unordered_set<unsigned short>, boost::hash<std::unordered_set<unsigned short>>> get_connected_points(const std::vector<unsigned short> &negative_points);
    std::pair<std::vector<Point3<DistanceType>>, std::vector<unsigned int>> get_internal_triangles(const std::unordered_set<std::unordered_set<unsigned short>, boost::hash<std::unordered_set<unsigned short>>>& connected_points, const std::vector<std::tuple<Vertex, Edge, Point3<DistanceType>>>& interpolation_points);
    std::vector<std::tuple<Vertex, Edge, Point3<DistanceType>>> get_interpolation_points(const std::vector<unsigned short>& negative_points);
public:
    std::vector<DistanceType> distances;
    std::vector<Point3<DistanceType>> points;
    static std::unordered_map<std::unordered_set<unsigned short>, unsigned short, boost::hash<std::unordered_set<unsigned short>>> neighborhood_distance;
    static std::unordered_map<unsigned short, std::unordered_set<unsigned short>> face_vertex;
    static std::unordered_map<unsigned short, std::unordered_set<unsigned short>> face_edge;
    static std::unordered_map<std::unordered_set<unsigned short>, unsigned short, boost::hash<std::unordered_set<unsigned short>>> vertex_edge;
    static std::unordered_map<unsigned short, std::unordered_set<unsigned short>> edge_vertex;
    static std::unordered_map<std::unordered_set<unsigned short>, unsigned short, boost::hash<std::unordered_set<unsigned short>>> vertex_face;
    static std::unordered_map<unsigned short, std::bitset<3>> neighbor_direction;
    static std::unordered_map<unsigned short, std::vector<unsigned short>> vertex_neighbor;
};

template<typename DistanceType>
std::unordered_map<std::unordered_set<unsigned short>, unsigned short, boost::hash<std::unordered_set<unsigned short>>> Cube<DistanceType>::neighborhood_distance = {
    {std::unordered_set<unsigned short>{0, 1}, 1},
    {std::unordered_set<unsigned short>(0, 2), 2},
    {std::unordered_set<unsigned short>(0, 3), 1},
    {std::unordered_set<unsigned short>(0, 4), 1},
    {std::unordered_set<unsigned short>(0, 5), 2},
    {std::unordered_set<unsigned short>(0, 6), 3},
    {std::unordered_set<unsigned short>(0, 7), 2},

    {std::unordered_set<unsigned short>(1, 2), 1},
    {std::unordered_set<unsigned short>(1, 3), 2},
    {std::unordered_set<unsigned short>(1, 4), 2},
    {std::unordered_set<unsigned short>(1, 5), 1},
    {std::unordered_set<unsigned short>(1, 6), 2},
    {std::unordered_set<unsigned short>(1, 7), 3},

    {std::unordered_set<unsigned short>(2, 3), 1},
    {std::unordered_set<unsigned short>(2, 4), 3},
    {std::unordered_set<unsigned short>(2, 5), 2},
    {std::unordered_set<unsigned short>(2, 6), 1},
    {std::unordered_set<unsigned short>(2, 7), 2},

    {std::unordered_set<unsigned short>(3, 4), 2},
    {std::unordered_set<unsigned short>(3, 5), 3},
    {std::unordered_set<unsigned short>(3, 6), 2},
    {std::unordered_set<unsigned short>(3, 7), 1},

    {std::unordered_set<unsigned short>(4, 5), 1},
    {std::unordered_set<unsigned short>(4, 6), 2},
    {std::unordered_set<unsigned short>(4, 7), 1},

    {std::unordered_set<unsigned short>(5, 6), 1},
    {std::unordered_set<unsigned short>(5, 7), 2},

    {std::unordered_set<unsigned short>(6, 7), 1},
};

template<typename DistanceType>
std::unordered_map<unsigned short, std::unordered_set<unsigned short>> Cube<DistanceType>::face_vertex = {
    {0, std::unordered_set<unsigned short>{0, 1, 5, 4}},
    {1, std::unordered_set<unsigned short>{1, 2, 6, 5}},
    {2, std::unordered_set<unsigned short>{2, 3, 7, 6}},
    {3, std::unordered_set<unsigned short>{0, 3, 7, 4}},
    {4, std::unordered_set<unsigned short>{0, 1, 2, 3}},
    {5, std::unordered_set<unsigned short>{4, 5, 6, 7}},
};

template<typename DistanceType>
std::unordered_map<unsigned short, std::unordered_set<unsigned short>> Cube<DistanceType>::face_edge = {
    {0, std::unordered_set<unsigned short>{0, 8, 4, 9}},
    {1, std::unordered_set<unsigned short>{1, 5, 9, 10}},
    {2, std::unordered_set<unsigned short>{2, 6, 10, 11}},
    {3, std::unordered_set<unsigned short>{3, 8, 7, 11}},
    {4, std::unordered_set<unsigned short>{0, 1, 2, 3}},
    {5, std::unordered_set<unsigned short>{4, 5, 6, 7}},
};

template<typename DistanceType>
std::unordered_map<std::unordered_set<unsigned short>, unsigned short, boost::hash<std::unordered_set<unsigned short>>> Cube<DistanceType>::vertex_edge = {
    {std::unordered_set<unsigned short>{0, 1}, 0},
    {std::unordered_set<unsigned short>{1, 2}, 1},
    {std::unordered_set<unsigned short>{2, 3}, 2},
    {std::unordered_set<unsigned short>{3, 0}, 3},
    {std::unordered_set<unsigned short>{4, 5}, 4},
    {std::unordered_set<unsigned short>{5, 6}, 5},
    {std::unordered_set<unsigned short>{6, 7}, 6},
    {std::unordered_set<unsigned short>{7, 4}, 7},
    {std::unordered_set<unsigned short>{0, 4}, 8},
    {std::unordered_set<unsigned short>{1, 5}, 9},
    {std::unordered_set<unsigned short>{2, 6}, 10},
    {std::unordered_set<unsigned short>{3, 7}, 11},
};

template<typename DistanceType>
std::unordered_map<unsigned short, std::unordered_set<unsigned short>> Cube<DistanceType>::edge_vertex = {
    {0, std::unordered_set<unsigned short>{0, 1}},
    {1, std::unordered_set<unsigned short>{1, 2}},
    {2, std::unordered_set<unsigned short>{2, 3}},
    {3, std::unordered_set<unsigned short>{3, 0}},
    {4, std::unordered_set<unsigned short>{4, 5}},
    {5, std::unordered_set<unsigned short>{5, 6}},
    {6, std::unordered_set<unsigned short>{6, 7}},
    {7, std::unordered_set<unsigned short>{7, 4}},
    {8, std::unordered_set<unsigned short>{0, 4}},
    {9, std::unordered_set<unsigned short>{1, 5}},
    {10, std::unordered_set<unsigned short>{2, 6}},
    {11, std::unordered_set<unsigned short>{3, 7}},
};

template<typename DistanceType>
std::unordered_map<std::unordered_set<unsigned short>, unsigned short, boost::hash<std::unordered_set<unsigned short>>> Cube<DistanceType>::vertex_face = {
    {std::unordered_set<unsinged short>{0, 5}, 0},
    {std::unordered_set<unsinged short>{1, 4}, 0},
    {std::unordered_set<unsinged short>{1, 6}, 1},
    {std::unordered_set<unsinged short>{2, 5}, 1},
    {std::unordered_set<unsinged short>{2, 7}, 2},
    {std::unordered_set<unsinged short>{3, 6}, 2},
    {std::unordered_set<unsinged short>{0, 7}, 3},
    {std::unordered_set<unsinged short>{3, 4}, 3},
    {std::unordered_set<unsinged short>{0, 2}, 4},
    {std::unordered_set<unsinged short>{1, 3}, 4},
    {std::unordered_set<unsinged short>{4, 6}, 5},
    {std::unordered_set<unsinged short>{5, 7}, 5},
};

template<typename DistanceType>
std::unordered_map<unsigned short, std::bitset<3>> Cube<DistanceType>::neighbor_direction = {
    {0, std::bitset<3>{0b111}},
    {1, std::bitset<3>{0b101}},
    {2, std::bitset<3>{0b001}},
    {3, std::bitset<3>{0b011}},
    {4, std::bitset<3>{0b110}},
    {5, std::bitset<3>{0b100}},
    {6, std::bitset<3>{0b000}},
    {7, std::bitset<3>{0b010}},
};

template<typename DistanceType>
std::unordered_map<unsigned short, std::vector<unsigned short>> Cube<DistanceType>::vertex_neighbor = {
    {0, {4, 1, 3}},
    {1, {5, 0, 2}},
    {2, {6, 3, 1}},
    {3, {7, 2, 0}},
    {4, {0, 5, 7}},
    {5, {1, 4, 6}},
    {6, {2, 7, 5}},
    {7, {3, 6, 4}},
};

template<typename DistanceType>
Cube<DistanceType>::Cube(std::vector<DistanceType> &distances, std::vector<Point3<DistanceType>> &points)
{
    this->distances = std::move(distances);
    this->points = std::move(points);
}

template<typename DistanceType>
std::vector<unsigned short> Cube<DistanceType>::get_negative_points()
{
    std::vector<unsigned short> indexes;
    for(auto iter = this->distances.begin(); iter != this->distances.end(); iter++)
    {
        if(*iter < 0.0f)
        {
            int index = std::distance(this->distances.begin(), iter);
            indexes.emplace_back(index);
        }
    }
    return indexes;
}

template<typename DistanceType>
std::unordered_set<std::unordered_set<unsigned short>, boost::hash<std::unordered_set<unsigned short>>> Cube<DistanceType>::get_connected_points(const std::vector<unsigned short> &negative_points)
{
    // std::vector<std::unordered_set<unsigned short>> connected_points;
    std::unordered_set<std::unordered_set<unsigned short>, boost::hash<std::unordered_set<unsigned short>>> connected_points;
    for(unsigned int i = 0; i < negative_points.size(); i++)
    {
        for(unsigned int j = 0; j < negative_points.size(); j++)
        {
            auto found = Cube<DistanceType>::neighborhood_distance.find(std::unordered_set<unsigned short>{negative_points[i], negative_points[j]});
            // TODO: if not found?
            auto distance = found->second;
            switch (distance)
            {
            case 1:
                // distance is 1, must connected
                connected_points.emplace(std::unordered_set<unsigned short>{negative_points[i], negative_points[j]});
                break;
            case 2:
                // distance is 2, same face, should consider face intersection is intersected
                // 1.  get the face of the two points
                auto face = Cube<DistanceType>::vertex_face.find(std::unordered_set<unsigned short>{negative_points[i], negative_points[j]})->second;
                // 2. get another two points
                auto other_two_points = Cube<DistanceType>::face_vertex.find(face)->second;
                other_two_points.erase(negative_points[i]);
                other_two_points.erase(negative_points[j]);
                // 3. calculate if intersect
                DistanceType AC = this->distances[negative_points[i]] * this->distances[negative_points[j]];
                DistanceType BD{1};
                for(const auto& elem: other_two_points)
                {
                    BD = BD * elem;
                }
                if(AC - BD > 0)
                {
                    // is connected
                    connected_points.emplace(std::unordered_set<unsigned short>{negative_points[i], negative_points[j]});
                }
                break;
            case 3:
                // distance is 3, test if cubic intersection is intersected
                // 1.  get the face of the two points
                auto face = Cube<DistanceType>::vertex_face.find(std::unordered_set<unsigned short>{negative_points[i], negative_points[j]})->second;
                // 2. get another two points
                auto other_two_points = Cube<DistanceType>::face_vertex.find(face)->second;
                other_two_points.erase(negative_points[i]);
                other_two_points.erase(negative_points[j]);
                auto B = *(other_two_points.begin());
                auto D = *(++other_two_points.begin());
                auto A = negative_points[i];
                auto C = negative_points[j];
                auto direction_a = Cube<DistanceType>::neighbor_direction[A];
                auto direction_c = Cube<DistanceType>::neighbor_direction[C];
                auto direction_b = Cube<DistanceType>::neighbor_direction[B];
                auto direction_d = Cube<DistanceType>::neighbor_direction[D];
                auto direction_size = direction_a.size();
                unsigned short same_direction = 0;
                for(unsigned short k = 0; k < direction_size; k++)
                {
                    if(direction_a[k] == direction_b[k] && direction_a[k] == direction_c[k] && direction_a[k] == direction_d[k])
                    {
                        same_direction = k;
                        break;
                    }
                }
                auto A1 = Cube<DistanceType>::vertex_neighbor[A][same_direction];
                auto B1 = Cube<DistanceType>::vertex_neighbor[B][same_direction];
                auto C1 = Cube<DistanceType>::vertex_neighbor[C][same_direction];
                auto D1 = Cube<DistanceType>::vertex_neighbor[D][same_direction];
                auto distance_a = this->distances[A];
                auto distance_a1 = this->distances[A1];
                auto distance_b = this->distances[B];
                auto distance_b1 = this->distances[B1];
                auto distance_c = this->distances[C];
                auto distance_c1 = this->distances[C1];
                auto distance_d = this->distances[D];
                auto distance_d1 = this->distances[D1];
                auto a = (distance_a1 - distance_a)*(distance_c1 - distance_c) - (distance_b1 - distance_b) * (distance_d1 - distance_d);
                auto b = distance_c*(distance_a1 - distance_a) + distance_a*(distance_c1 - distance_c) - distance_d*(distance_b1 - distance_b) - distance_b*(distance_d1 - distance_d);
                auto c = distance_a*distance_c - distance_b*distance_d;
                if(a < 0)
                {
                    auto t_max = -b / (2 * a);
                    if(a * std::pow(t_max, 2) + b * t_max + c > 0)
                    {
                        // joined
                        connected_points.emplace(std::unordered_set<unsigned short>{negative_points[i], negative_points[j]});
                    }
                }
                break;
            default:
                break;
            }
        }
    }
    return connected_points;
}

template<typename DistanceType>
std::vector<std::tuple<Vertex, Edge, Point3<DistanceType>>> Cube<DistanceType>::get_interpolation_points(const std::vector<unsigned short>& negative_points)
{
    std::vector<std::tuple<Vertex, Edge, Point3<DistanceType>>> interpolation_points;
    for(const auto& negative_point: negative_points)
    {
        auto neighbors = Cube<DistanceType>::vertex_neighbor[negative_point];
        // for(auto& neighbor: neighbors)
        for(auto neighbor_iter = neighbors.begin(); neighbor_iter != neighbors.end(); neighbor_iter++)
        {
            auto neighbor = *neighbor_iter;
            int index = std::distance(neighbors.begin(), neighbor_iter);
            auto nb_dir = Cube<DistanceType>::neighbor_direction[negative_point][index];
            if(this->distances[neighbor] >= 0)
            {
                // can interpolate point between these two points
                Edge edge = Edge(Cube<DistanceType>::vertex_edge.find({negative_point, neighbor})->second);
                switch(edge)
                {
                // x direction
                case Edge::e8, Edge::e9, Edge::ea, Edge::eb:
                    auto y_coor = this->points[neighbor].y;
                    auto z_coor = this->points[neighbor].z;
                    auto t = (this->distances[negative_point]) / (this->distances[negative_point] - this->distances[neighbor]);
                    auto x_coor = this->points[negative_point].x;
                    if(nb_dir)
                    {
                        x_coor = x_coor + t * std::abs(this->points[neighbor].x - this->points[negative_point].x);
                    }
                    else
                    {
                        x_coor = x_coor - t * std::abs(this->points[neighbor].x - this->points[negative_point].x);
                    }
                    interpolation_points.emplace_back(std::make_tuple(negative_point, edge, Point3<DistanceType>{x_coor, y_coor, z_coor}));
                    break;
                // y direction
                case Edge::e0, Edge::e2, Edge::e4, Edge::e6:
                    auto x_coor = this->points[neighbor].x;
                    auto z_coor = this->points[neighbor].y;
                    auto t = (this->distances[negative_point]) / (this->distances[negative_point] - this->distances[neighbor]);
                    auto y_coor = this->points[negative_point].y;
                    if(nb_dir)
                    {
                        y_coor = y_coor + t * std::abs(this->points[neighbor].y - this->points[negative_point].y);
                    }
                    else
                    {
                        y_coor = y_coor - t * std::abs(this->points[neighbor].y - this->points[negative_point].y);
                    }
                    interpolation_points.emplace_back(std::make_tuple(negative_point, edge, Point3<DistanceType>{x_coor, y_coor, z_coor}));
                    break;
                // z direction
                case Edge::e1, Edge::e3, Edge::e5, Edge::e7:
                    auto y_coor = this->points[neighbor].y;
                    auto x_coor = this->points[neighbor].x;
                    auto t = (this->distances[negative_point]) / (this->distances[negative_point] - this->distances[neighbor]);
                    auto z_coor = this->points[negative_point].z;
                    if(nb_dir)
                    {
                        z_coor = z_coor + t * std::abs(this->points[neighbor].z - this->points[negative_point].z);
                    }
                    else
                    {
                        z_coor = z_coor - t * std::abs(this->points[neighbor].z - this->points[negative_point].z);
                    }
                    interpolation_points.emplace_back(std::make_tuple(negative_point, edge, Point3<DistanceType>{x_coor, y_coor, z_coor}));
                    break;
                default:
                    break;
                }
            }
        }
    }
    return interpolation_points;
}

template<typename DistanceType>
std::pair<std::vector<Point3<DistanceType>>, std::vector<unsigned int>> Cube<DistanceType>::get_internal_triangles(
    const std::unordered_set<std::unordered_set<unsigned short>, boost::hash<std::unordered_set<unsigned short>>>& connected_points,
    const std::vector<std::tuple<Vertex, Edge, Point3<DistanceType>>>& interpolation_points
)
{

}


#endif