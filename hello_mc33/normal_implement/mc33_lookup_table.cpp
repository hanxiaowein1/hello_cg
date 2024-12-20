#include "mc33_lookup_table.h"
#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <bitset>
#include <queue>
#include <stdexcept>
#include "mc33_global_test.h"
#include <format>
/**
 * @brief first level deside big case, second level deside divided case
 * first level key is distance_sign, second level is connected vertices of ambiguous
 * 
 */
std::unordered_map<
    std::bitset<8>,
    std::unordered_map<
        std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>,
        std::vector<unsigned short>,
        boost::hash<std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>>
    >
> MC33_TABLES = {
    // case 0
    {
        0b11111111,
        {
            {
                {}, {}
            }
        }
    },

    // case 1
    {
        0b11101111,
        {
            {
                {}, {0x487}
            }
        }
    },

    // case 2
    {
        0b11001111,
        {
            {
                {}, {0x875, 0x985}
            }
        }
    },

    // case 3
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

    // case 4
    {
        0b11101011,
        {
            // case 4.1.1
            {
                {}, {0x487, 0x1A2}
            },
            // case 4.1.2
            {
                {{Vertex::v2, Vertex::v4}}, {0x27A, 0xA74, 0xA41, 0x148, 0x218, 0x287}
            }
        }
    },

    // case 5
    {
        0b11011100,
        {
            {
                {}, {0x351, 0x385, 0x845}
            }
        }
    },

    // case 6
    {
        0b11001011,
        {
            // case 6.1.1
            {
                {{Vertex::v1, Vertex::v6}}, {0x758, 0x859, 0x1A2}
            },
            // case 6.1.2
            {
                {{Vertex::v1, Vertex::v6}, {Vertex::v2, Vertex::v4}}, {0x59A, 0xA91, 0x198, 0x182, 0x287, 0x27A, 0xA75}
            },
            {
                {{Vertex::v2, Vertex::v4}, {Vertex::v2, Vertex::v5}}, {0x981, 0x182, 0x287, 0x27A, 0xA75}
            }
        }
    },

    // case 7
    {
        0b01011011,
        {
            // case 7.1
            {
                {{Vertex::v3, Vertex::v6}, {Vertex::v4, Vertex::v6}, {Vertex::v1, Vertex::v6}, {Vertex::v0, Vertex::v6}}, {0x1A2, 0x37B, 0x459},
            },
            // case 7.2
            {
                {{Vertex::v2, Vertex::v7}, {Vertex::v4, Vertex::v6}, {Vertex::v1, Vertex::v6}, {Vertex::v0, Vertex::v6}},
                {0xB27, 0x721, 0x716, 0x61A, 0x459}
            },
            // case 7.3
            {
                {{Vertex::v2, Vertex::v7}, {Vertex::v2, Vertex::v5}, {Vertex::v4, Vertex::v6}, {Vertex::v0, Vertex::v6}},
                // C meanings points interpolated in middle of cube
                {0xC7B, 0xCB2, 0xC21, 0xC19, 0xC94, 0xC45, 0xC5A, 0xCA6, 0xC67},
            },
            // case 7.4.1
            {
                {{Vertex::v2, Vertex::v7}, {Vertex::v2, Vertex::v5}, {Vertex::v5, Vertex::v7}},
                {0x47B, 0x419, 0x21B, 0xB14, 0x5A6}
            },
            // case 7.4.2
            {
                {{Vertex::v2, Vertex::v7}, {Vertex::v2, Vertex::v5}, {Vertex::v5, Vertex::v7}, {Vertex::v0, Vertex::v6}},
                {0x67B, 0x657, 0x547, 0x459, 0x519, 0x5A1, 0x1A2, 0x2A6, 0x26B}
            }
        }
    },

    // case 8
    {
        0b11001100,
        {
            {
                {}, {0x137, 0x175}
            }
        }
    },

    // case 9
    {
        0b11100100,
        {
            {
                {}, {0x2B7, 0x274, 0x241, 0x149}
            }
        }
    },

    // case 10
    {
        0b01101001,
        {
            // case 10.1.1
            {
                {{Vertex::v3, Vertex::v5}, {Vertex::v0, Vertex::v5}, {Vertex::v0, Vertex::v6}, {Vertex::v3, Vertex::v5}},
                {0x20A, 0x09A, 0x68B, 0x648}
            },
            // case 10.1.2
            {
                {{Vertex::v3, Vertex::v5}, {Vertex::v0, Vertex::v2}, {Vertex::v2, Vertex::v4}, {Vertex::v1, Vertex::v7}},
                {0xB6A, 0x2BA, 0x20B, 0xB08, 0x048, 0x409, 0x64A, 0x49A}
            },
            // case 10.2
            {
                {{Vertex::v0, Vertex::v5}, {Vertex::v1, Vertex::v4}, {Vertex::v2, Vertex::v4}, {Vertex::v1, Vertex::v7}},
                {0xCA2, 0xC9A, 0xC49, 0xC64, 0xCB6, 0xC8B, 0xC08}
            }
        }
    },

    // case 11
    {
        0b11101000,
        {
            {
                {}, {0x23A, 0xA34, 0x374, 0xA49}
            }
        }
    },

    // case 12
    {
        0b01011100,
        {
            // case 12.1.1
            {
                {{Vertex::v3, Vertex::v4}, {Vertex::v4, Vertex::v6}}, {0x67B, 0x351, 0x385, 0x845}
            },
            // case 12.1.2
            {
                {{Vertex::v3, Vertex::v4}, {Vertex::v4, Vertex::v6}, {Vertex::v1, Vertex::v7}},
                {0x38B, 0xB87, 0x478, 0x467, 0x456, 0x516, 0xB61, 0xB13}
            },
            // case 12.2
            {
                {{Vertex::v3, Vertex::v4}, {Vertex::v5, Vertex::v7}, {Vertex::v1, Vertex::v7}},
                {0xC51, 0xC13, 0xC38, 0xC84, 0xC47, 0xC7B, 0xCB6, 0xC65}
            },
            // case 12.3
            {
                {{Vertex::v4, Vertex::v6}, {Vertex::v2, Vertex::v4}, {Vertex::v0, Vertex::v7}},
                {0xC84, 0xC45, 0xC51, 0xC13, 0xC3B, 0xCB6, 0xC67, 0xC78}
            }
        }
    },

    // case 13
    {
        0b10100101,
        {
            // case 13.1
            {
                {{Vertex::v2, Vertex::v7}, {Vertex::v0, Vertex::v5}, {Vertex::v0, Vertex::v7}, {Vertex::v2, Vertex::v5}, {Vertex::v5, Vertex::v7}, {Vertex::v0, Vertex::v2}},
                {0x019, 0x2B3, 0x487, 0x56A}
            },
            // case 13.2
            {
                {{Vertex::v3, Vertex::v6}, {Vertex::v0, Vertex::v5}, {Vertex::v0, Vertex::v7}, {Vertex::v2, Vertex::v5}, {Vertex::v5, Vertex::v7}, {Vertex::v0, Vertex::v2}},
                {0x487, 0x091, 0x2A5, 0x253, 0x35B, 0xB56}
            },
            // case 13.3
            {
                {{Vertex::v3, Vertex::v6}, {Vertex::v0, Vertex::v5}, {Vertex::v3, Vertex::v4}, {Vertex::v2, Vertex::v5}, {Vertex::v5, Vertex::v7}, {Vertex::v0, Vertex::v2}},
                {0xCA5, 0xC2A, 0xC32, 0xC83, 0xC48, 0xC74, 0xCB7, 0xC6B, 0xC56, 0x091}
            },
            // case 13.4
            {
                {{Vertex::v0, Vertex::v6}, {Vertex::v0, Vertex::v5}, {Vertex::v3, Vertex::v4}, {Vertex::v2, Vertex::v5}, {Vertex::v5, Vertex::v7}, {Vertex::v1, Vertex::v3}},
                {0xC30, 0xC83, 0xC48, 0xC47, 0xCB7, 0xC6B, 0xC56, 0xCA5, 0xC2A, 0xC12, 0xC91, 0xC09}
            },
            // case 13.5.1
            {
                {{Vertex::v3, Vertex::v6}, {Vertex::v0, Vertex::v5}, {Vertex::v0, Vertex::v7}, {Vertex::v1, Vertex::v6}, {Vertex::v5, Vertex::v7}, {Vertex::v1, Vertex::v3}},
                {0x487, 0x12A, 0xB36, 0x306, 0x605, 0x095}
            },
            // case 13.5.2
            {
                {{Vertex::v3, Vertex::v5}, {Vertex::v1, Vertex::v4}, {Vertex::v0, Vertex::v7}, {Vertex::v1, Vertex::v6}, {Vertex::v5, Vertex::v7}, {Vertex::v1, Vertex::v3}},
                {0x12A, 0x480, 0x803, 0x387, 0x37B, 0x76B, 0x674, 0x645, 0x495, 0x094}
            }
        }
    },

    // case 14
    {
        0b11010100,
        {
            {
                {}, {0x4B8, 0x41B, 0x451, 0xB12}
            }
        }
    }
};

std::unordered_map<Vertex, std::unordered_set<Vertex>> VERTEX_NEIGHBOOR = {
    {Vertex::v0, {Vertex::v1, Vertex::v3, Vertex::v4}},
    {Vertex::v1, {Vertex::v0, Vertex::v2, Vertex::v5}},
    {Vertex::v2, {Vertex::v1, Vertex::v3, Vertex::v6}},
    {Vertex::v3, {Vertex::v0, Vertex::v2, Vertex::v7}},
    {Vertex::v4, {Vertex::v0, Vertex::v5, Vertex::v7}},
    {Vertex::v5, {Vertex::v1, Vertex::v4, Vertex::v6}},
    {Vertex::v6, {Vertex::v2, Vertex::v5, Vertex::v7}},
    {Vertex::v7, {Vertex::v3, Vertex::v4, Vertex::v6}},
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
    {{Vertex::vc1, Vertex::vc2}, Edge::ec},
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
    {Edge::ec, {Vertex::vc1, Vertex::vc2}},
};

std::unordered_map<std::unordered_set<Vertex>, unsigned short, boost::hash<std::unordered_set<Vertex>>> NEIGHBOR_DISTANCE = {
    {{Vertex::v0, Vertex::v1}, 1},
    {{Vertex::v0, Vertex::v2}, 2},
    {{Vertex::v0, Vertex::v3}, 1},
    {{Vertex::v0, Vertex::v4}, 1},
    {{Vertex::v0, Vertex::v5}, 2},
    {{Vertex::v0, Vertex::v6}, 3},
    {{Vertex::v0, Vertex::v7}, 2},

    {{Vertex::v1, Vertex::v2}, 1},
    {{Vertex::v1, Vertex::v3}, 2},
    {{Vertex::v1, Vertex::v4}, 2},
    {{Vertex::v1, Vertex::v5}, 1},
    {{Vertex::v1, Vertex::v6}, 2},
    {{Vertex::v1, Vertex::v7}, 3},

    {{Vertex::v2, Vertex::v3}, 1},
    {{Vertex::v2, Vertex::v4}, 3},
    {{Vertex::v2, Vertex::v5}, 2},
    {{Vertex::v2, Vertex::v6}, 1},
    {{Vertex::v2, Vertex::v7}, 2},

    {{Vertex::v3, Vertex::v4}, 2},
    {{Vertex::v3, Vertex::v5}, 3},
    {{Vertex::v3, Vertex::v6}, 2},
    {{Vertex::v3, Vertex::v7}, 1},

    {{Vertex::v4, Vertex::v5}, 1},
    {{Vertex::v4, Vertex::v6}, 2},
    {{Vertex::v4, Vertex::v7}, 1},

    {{Vertex::v5, Vertex::v6}, 1},
    {{Vertex::v5, Vertex::v7}, 2},

    {{Vertex::v6, Vertex::v7}, 1},
};

std::unordered_map<std::unordered_set<Vertex>, Face, boost::hash<std::unordered_set<Vertex>>> DIAGNAL_VERTEX_FACE = {
    {{Vertex::v0, Vertex::v5}, Face::f0},
    {{Vertex::v1, Vertex::v4}, Face::f0},
    {{Vertex::v1, Vertex::v6}, Face::f1},
    {{Vertex::v2, Vertex::v5}, Face::f1},
    {{Vertex::v2, Vertex::v7}, Face::f2},
    {{Vertex::v3, Vertex::v6}, Face::f2},
    {{Vertex::v0, Vertex::v7}, Face::f3},
    {{Vertex::v3, Vertex::v4}, Face::f3},
    {{Vertex::v0, Vertex::v2}, Face::f4},
    {{Vertex::v1, Vertex::v3}, Face::f4},
    {{Vertex::v4, Vertex::v6}, Face::f5},
    {{Vertex::v5, Vertex::v7}, Face::f5},
};

std::unordered_map<Face, std::unordered_set<Vertex>> FACE_VERTICES = {
    {Face::f0, {Vertex::v0, Vertex::v1, Vertex::v5, Vertex::v4}},
    {Face::f1, {Vertex::v1, Vertex::v2, Vertex::v6, Vertex::v5}},
    {Face::f2, {Vertex::v2, Vertex::v3, Vertex::v7, Vertex::v6}},
    {Face::f3, {Vertex::v0, Vertex::v3, Vertex::v7, Vertex::v4}},
    {Face::f4, {Vertex::v0, Vertex::v1, Vertex::v2, Vertex::v3}},
    {Face::f5, {Vertex::v4, Vertex::v5, Vertex::v6, Vertex::v7}},
};

Face TOP_FACE = Face::f2;

Face BOTTOM_FACE = Face::f0;

std::unordered_map<std::unordered_set<Vertex>, std::unordered_map<std::string, Vertex>, boost::hash<std::unordered_set<Vertex>>> CUBE_INTERPOLATION_MAP = {
    {
        {Vertex::v0, Vertex::v6},
        {
            {"A0", Vertex::v0},
            {"C0", Vertex::v5},
            {"B0", Vertex::v1},
            {"D0", Vertex::v4},
            {"A1", Vertex::v3},
            {"C1", Vertex::v6},
            {"B1", Vertex::v2},
            {"D1", Vertex::v7}
        }
    },
    {
        {Vertex::v1, Vertex::v7},
        {
            {"A0", Vertex::v1},
            {"C0", Vertex::v4},
            {"B0", Vertex::v5},
            {"D0", Vertex::v0},
            {"A1", Vertex::v2},
            {"C1", Vertex::v7},
            {"B1", Vertex::v6},
            {"D1", Vertex::v3}
        }
    },
    {
        {Vertex::v5, Vertex::v3},
        {
            {"A0", Vertex::v5},
            {"C0", Vertex::v0},
            {"B0", Vertex::v4},
            {"D0", Vertex::v1},
            {"A1", Vertex::v6},
            {"C1", Vertex::v3},
            {"B1", Vertex::v7},
            {"D1", Vertex::v2}
        }
    },
    {
        {Vertex::v4, Vertex::v2},
        {
            {"A0", Vertex::v4},
            {"C0", Vertex::v1},
            {"B0", Vertex::v0},
            {"D0", Vertex::v5},
            {"A1", Vertex::v7},
            {"C1", Vertex::v2},
            {"B1", Vertex::v3},
            {"D1", Vertex::v6}
        }
    }
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
    {Vertex::vc1, Vertex::vc1},
    {Vertex::vc2, Vertex::vc2},
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
    {Vertex::vc1, Vertex::vc1},
    {Vertex::vc2, Vertex::vc2},
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
    {Vertex::vc1, Vertex::vc1},
    {Vertex::vc2, Vertex::vc2},
};


bool vertex_in_top_face(const Vertex& vertex)
{
    auto vertices = FACE_VERTICES.at(TOP_FACE);
    if(vertices.contains(vertex))
    {
        return true;
    }
    return false;
}

bool vertex_in_bottom_face(const Vertex& vertex)
{
    auto vertices = FACE_VERTICES.at(BOTTOM_FACE);
    if(vertices.contains(vertex))
    {
        return true;
    }
    return false;
}

bool has_value_bigger_than_zero_in_interval(
    const double& a, const double& b, const double& c,
    const double& start = 0.0f, const double& end = 1.0f
)
{
    auto fx = [](double a, double b, double c, double x) -> double{
        return a * std::pow(x, 2) + b * x + c;
    };
    if(a >= 0.0f)
    {
        if(fx(a, b, c, start) > 0 || fx(a, b, c, end) > 0)
        {
            return true;
        }
        return false;
    }
    else
    {
        // must has root, so(b^2 - 4ac > 0)
        if(std::pow(b, 2) - 4.0f * a * c > 0)
        {
            // if (fx(start) > 0) or (fx(end) > 0) or (-b/2a between start and end)
            if(fx(a, b, c, start) > 0 || fx(a, b, c, end) > 0 || (-b / (2.0f * a) > start && -b / (2.0f * a) < end))
            {
                return true;
            }
            return false;
        }
        return false;
    }
}

/**
 * @brief suppose vertex1 and vertex2 are not obviously connected
 * 
 * @param signed_distance 
 * @param vertex1 
 * @param vertex2 
 * @return true 
 * @return false 
 */
bool vertex_interpolation_connected(const std::vector<double>& signed_distance, const Vertex& vertex1, const Vertex& vertex2)
{
    // only has two cases, one is in the diagonal in same face, one is in diagonal of the cube
    // check if two vertex is one the same face(easy, check distance will be fine)
    auto distance = NEIGHBOR_DISTANCE.at(std::unordered_set<Vertex>{vertex1, vertex2});
    switch(distance)
    {
        case 1:
            // distance is 1, is same edge, must be connected
            return true;
        case 2:
        {
            // distance is 2, is diagnal point in same face, consider interpolation connected
            auto face = DIAGNAL_VERTEX_FACE.at(std::unordered_set<Vertex>{vertex1, vertex2});
            auto other_two_points = FACE_VERTICES.at(face);
            other_two_points.erase(vertex1);
            other_two_points.erase(vertex2);
            double distance_ac = signed_distance[static_cast<unsigned int>(vertex1)] * signed_distance[static_cast<unsigned int>(vertex2)];
            double distance_bd = 1.0f;
            for(const auto& elem: other_two_points)
            {
                distance_bd = distance_bd * signed_distance[static_cast<unsigned int>(elem)];
            }
            if(distance_ac - distance_bd > 0.0f)
            {
                return true;
            }
            break;
        }
        case 3:
        {
            // distance is 3, is diagnal point in cube, just compute the top and bottom interpolation
            auto cube_interpolation = CUBE_INTERPOLATION_MAP.at(std::unordered_set<Vertex>{vertex1, vertex2});
            double distance_a0 = signed_distance[static_cast<unsigned short>(cube_interpolation["A0"])];
            double distance_a1 = signed_distance[static_cast<unsigned short>(cube_interpolation["A1"])];
            double distance_b0 = signed_distance[static_cast<unsigned short>(cube_interpolation["B0"])];
            double distance_b1 = signed_distance[static_cast<unsigned short>(cube_interpolation["B1"])];
            double distance_c0 = signed_distance[static_cast<unsigned short>(cube_interpolation["C0"])];
            double distance_c1 = signed_distance[static_cast<unsigned short>(cube_interpolation["C1"])];
            double distance_d0 = signed_distance[static_cast<unsigned short>(cube_interpolation["D0"])];
            double distance_d1 = signed_distance[static_cast<unsigned short>(cube_interpolation["D1"])];
            auto a = (distance_a1 - distance_a0)*(distance_c1 - distance_c0) - (distance_b1 - distance_b0) * (distance_d1 - distance_d0);
            auto b = distance_c0*(distance_a1 - distance_a0) + distance_a0*(distance_c1 - distance_c0) - distance_d0*(distance_b1 - distance_b0) - distance_b0*(distance_d1 - distance_d0);
            auto c = distance_a0*distance_c0 - distance_b0 * distance_d0;
            if(has_value_bigger_than_zero_in_interval(a, b, c, 0.0f, 1.0f))
            {
                return true;
            }
            // if(a < 0)
            // {
            //     auto t_max = -b / (2 * a);
            //     if(t_max > 0.0f && t_max < 1.0f)
            //     {
            //         if(a * std::pow(t_max, 2) + b * t_max + c > 0)
            //         {
            //             // joined
            //             return true;
            //         }
            //     }
            // }
            break;
        }
        default:
            break;
    }
    return false;
}

/**
 * @brief find if there are path between v1 and v2
 * 
 * @param vertex1 
 * @param vertex2 
 * @return true 
 * @return false 
 */
bool vertex_connected(const std::bitset<8>& distance_sign, const Vertex &vertex1, const Vertex &vertex2)
{
    // TODO: maybe need a cache to avoid duplicate calculation
    if(vertex1 == vertex2)
    {
        return true;
    }
    auto vertex1_symbol = distance_sign[static_cast<unsigned short>(vertex1)];
    auto vertex2_symbol = distance_sign[static_cast<unsigned short>(vertex2)];
    if(vertex1_symbol != vertex2_symbol)
    {
        return false;
    }
    auto vertex_symbol = vertex1_symbol;

    // avoid visited vertex
    std::unordered_set<Vertex> visited_vertices;
    std::queue<std::pair<Vertex, unsigned short>> visiting_vertices;
    visiting_vertices.push(std::make_pair(vertex1, 0));
    while (!visiting_vertices.empty())
    {
        auto current_vertex = visiting_vertices.front();
        visiting_vertices.pop();
        auto depth = current_vertex.second;
        if (depth >= 4)
        {
            return false;
        }
        if (current_vertex.first == vertex2)
        {
            return true;
        }
        auto neighboors = VERTEX_NEIGHBOOR.at(current_vertex.first);
        for(const auto& neighboor: neighboors)
        {
            // symbol is same
            if(distance_sign[static_cast<unsigned short>(neighboor)] == vertex_symbol)
            {
                if(!visited_vertices.contains(neighboor))
                {
                    visiting_vertices.push(std::make_pair(neighboor, depth+1));
                }
            }
        }
        visited_vertices.emplace(current_vertex.first);
    }
    return false;
}

Vertex rotate_vertex_in_axis(const Vertex& vertex, const Axis& axis, int times = 1)
{
    Vertex rotated_vertex = vertex;
    for(int i = 0; i < times; i++)
    {
        switch(axis)
        {
        case Axis::x:
            rotated_vertex = VERTEX_X_ROTATE.at(rotated_vertex);
            break;
        case Axis::y:
            rotated_vertex = VERTEX_Y_ROTATE.at(rotated_vertex);
            break;
        case Axis::z:
            rotated_vertex = VERTEX_Z_ROTATE.at(rotated_vertex);
            break;
        default:
            throw std::invalid_argument("axis is weird");
            break;
        }
    }
    return rotated_vertex;
}

std::vector<Vertex> rotate_vertices_in_axis(const std::vector<Vertex>& vertices, const Axis& axis, int times = 1)
{
    std::vector<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        rotated_vertex = rotate_vertex_in_axis(rotated_vertex, axis, times);
        rotated_vertices.emplace_back(rotated_vertex);
    }
    return rotated_vertices;
}

Edge rotate_edge_in_axis(const Edge& edge, const Axis& axis, int times = 1)
{
    auto vertices = EDGE_VERTEX.at(edge);
    std::unordered_set<Vertex> rotated_vertices;
    for(const auto& vertex: vertices)
    {
        auto rotated_vertex = vertex;
        for(int i = 0; i < times; i++)
        {
            switch (axis)
            {
            case Axis::x:
                rotated_vertex = VERTEX_X_ROTATE.at(rotated_vertex);
                break;
            case Axis::y:
                rotated_vertex = VERTEX_Y_ROTATE.at(rotated_vertex);
                break;
            case Axis::z:
                rotated_vertex = VERTEX_Z_ROTATE.at(rotated_vertex);
                break;
            default:
                throw std::invalid_argument("axis is weird");
                break;
            }
        }
        rotated_vertices.emplace(rotated_vertex);
    }
    auto rotated_edge = VERTEX_EDGE.at(rotated_vertices);
    return rotated_edge;
}

std::vector<unsigned short> rotate_edge_in_axis(const std::vector<unsigned short>& edgess, const Axis& axis, int times = 1)
{
    std::vector<unsigned short> rotated_edgess;
    for(auto edges: edgess)
    {
        unsigned short rotated_edges = 0x0000;
        for(int i = 0; i < 3; i++)
        {
            edges = edges >> (i == 0 ? 0 : 4);
            auto edge = edges & 0xF;
            Edge rotated_edge;
            rotated_edge = rotate_edge_in_axis(static_cast<Edge>(edge), axis, times);
            rotated_edges = (rotated_edges | (static_cast<unsigned short>(rotated_edge) << (4 * i)));
        }
        rotated_edgess.emplace_back(rotated_edges);
    }
    return rotated_edgess;
}

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

std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>> rotate_vertices_in_axis(
    const std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>& verticess,
    const Axis& axis, int times = 1
)
{
    std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>> rotated_verticess;
    for(const auto& connected_vertices: verticess)
    {
        std::unordered_set<Vertex> rotated_connnected_vertices;
        for(const auto& connected_vertex: connected_vertices)
        {
            Vertex rotated_connected_vertex = rotate_vertex_in_axis(connected_vertex, axis, times);
            rotated_connnected_vertices.emplace(rotated_connected_vertex);
        }
        rotated_verticess.emplace(rotated_connnected_vertices);
    }
    return rotated_verticess;
}

void init_tables()
{
    // rotate table
    std::unordered_map<
        std::bitset<8>,
        std::unordered_map<
            std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>,
            std::vector<unsigned short>,
            boost::hash<std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>>
        >
    > rotated_tables;
    for(auto [distance_signs, sub_cases]: MC33_TABLES)
    {
        // get negative vertex
        auto vertices = get_negative_vertices_from_signed_distance(distance_signs);
        for(auto [connected_verticess, triangles]: sub_cases)
        {
            for(unsigned short k = 0; k < 4; k++)
            {
                auto z_rotated_vertices = rotate_vertices_in_axis(vertices, Axis::z, k);
                auto z_rotated_edgess = rotate_edge_in_axis(triangles, Axis::z, k);
                auto z_rotated_connected_vertices = rotate_vertices_in_axis(connected_verticess, Axis::z, k);
                for(unsigned short j = 0; j < 4; j++)
                {
                    auto y_rotated_vertices = rotate_vertices_in_axis(z_rotated_vertices, Axis::y, j);
                    auto y_rotated_edgess = rotate_edge_in_axis(z_rotated_edgess, Axis::y, j);
                    auto y_rotated_connected_vertices = rotate_vertices_in_axis(z_rotated_connected_vertices, Axis::y, j);
                    for(unsigned short i = 0; i < 4; i++)
                    {
                        auto x_rotated_vertices = rotate_vertices_in_axis(y_rotated_vertices, Axis::x, i);
                        auto x_rotated_edgess = rotate_edge_in_axis(y_rotated_edgess, Axis::x, i);
                        auto x_rotated_connected_vertices = rotate_vertices_in_axis(y_rotated_connected_vertices, Axis::x, i);
                        // use rotated vertices to construct new signed distance
                        std::bitset<8> rotated_signed_distance{0b11111111};
                        for(const auto& x_rotated_vertice: x_rotated_vertices)
                        {
                            rotated_signed_distance.set(static_cast<int>(x_rotated_vertice), false);
                        }
                        if(!rotated_tables.contains(rotated_signed_distance))
                        {
                            std::unordered_map<
                                std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>,
                                std::vector<unsigned short>,
                                boost::hash<std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>>
                            > temp_map;
                            temp_map.emplace(std::make_pair(x_rotated_connected_vertices, x_rotated_edgess));
                            rotated_tables.emplace(std::make_pair(rotated_signed_distance, temp_map));
                        }
                        else
                        {
                            if(!rotated_tables.at(rotated_signed_distance).contains(x_rotated_connected_vertices))
                            {
                                rotated_tables.at(rotated_signed_distance).emplace(std::make_pair(x_rotated_connected_vertices, x_rotated_edgess));
                            }
                        }
                    }
                }
            }
        }
    }

    MC33_TABLES.insert(rotated_tables.begin(), rotated_tables.end());
    std::cout << "after rotate, mc33 table size is: " << MC33_TABLES.size() << std::endl;

    // invert distance signs
    std::unordered_map<
        std::bitset<8>,
        std::unordered_map<
            std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>,
            std::vector<unsigned short>,
            boost::hash<std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>>
        >
    > invert_table;
    for(auto [key, value]: MC33_TABLES)
    {
        auto origin = key;
        origin.flip();
        invert_table.emplace(std::make_pair(origin, value));
    }

    MC33_TABLES.insert(invert_table.begin(), invert_table.end());

    std::cout << "after invert, mc33 table size is: " << MC33_TABLES.size() << std::endl;
}

void print_mc33_table()
{
    for(const auto& [distances_signs, sub_cases]: MC33_TABLES)
    {
        std::cout << "{ distances_signs: "<< distances_signs << ", ";
        std::cout << "sub_cases: {";
        for(const auto& [connected_verticess, triangles]: sub_cases)
        {
            std::cout << "{" << "connected_vertices: {";
            for(const auto& connected_vertices: connected_verticess)
            {
                std::cout << "{";
                for(const auto& connected_vertex: connected_vertices)
                {
                    std::cout << static_cast<unsigned short>(connected_vertex) << " ";
                }
                std::cout << "},";
            }
            std::cout << "}, triangles:{";
            for(const auto& triangle: triangles)
            {
                std::cout << std::format("{:#x} ", triangle) << ", ";
            }
            std::cout << "}";
        }
        std::cout << "}," << std::endl;
    }
}

TEST(GlobalTest, init_tables)
{
    init_tables();
    print_mc33_table();
}

TEST(GlobalTest, vertex_interpolation_connected_cube_diagnal_connected)
{
    std::vector<double> signed_distance1{-100, 1, 1, 1, 1, 1, -100, 1};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v6;
    auto connected1 = vertex_interpolation_connected(signed_distance1, v1, v2);
    ASSERT_EQ(connected1, true);
}

TEST(GlobalTest, vertex_interpolation_connected_cube_diagnal_disconnected)
{
    std::vector<double> signed_distance1{-1, 100, 100, 100, 100, 100, -1, 100};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v6;
    auto connected1 = vertex_interpolation_connected(signed_distance1, v1, v2);
    ASSERT_EQ(connected1, false);
}

TEST(GlobalTest, vertex_interpolation_connected_face_diagnal_connected)
{
    std::vector<double> signed_distance1{-100, 1, -100, 101, 100, 100, -1, 100};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v2;
    auto connected1 = vertex_interpolation_connected(signed_distance1, v1, v2);
    ASSERT_EQ(connected1, true);
}

TEST(GlobalTest, vertex_interpolation_connected_face_diagnal_disconnected)
{
    std::vector<double> signed_distance1{-1, 100, -1, 100, 100, 100, -1, 100};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v2;
    auto connected1 = vertex_interpolation_connected(signed_distance1, v1, v2);
    ASSERT_EQ(connected1, false);
}

TEST(GlobalTest, vertex_connected_same_edge_connected)
{
    std::bitset<8> signed_distance{0b11111100};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v1;
    auto connected = vertex_connected(signed_distance, v1, v2);
    ASSERT_EQ(connected, true);
}

TEST(GlobalTest, vertex_connected_same_edge_disconnected)
{
    std::bitset<8> signed_distance{0b11111110};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v1;
    auto connected = vertex_connected(signed_distance, v1, v2);
    ASSERT_EQ(connected, false);
}

TEST(GlobalTest, vertex_connected_edge_diagnal_connected)
{
    std::bitset<8> signed_distance{0b11111000};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v2;
    auto connected = vertex_connected(signed_distance, v1, v2);
    ASSERT_EQ(connected, true);
}

TEST(GlobalTest, vertex_connected_edge_diagnal_disconnected)
{
    {
        std::bitset<8> signed_distance{0b11111010};
        Vertex v1 = Vertex::v0;
        Vertex v2 = Vertex::v2;
        auto connected = vertex_connected(signed_distance, v1, v2);
        ASSERT_EQ(connected, false);
    }

    {
        std::bitset<8> signed_distance{0b11001011};
        Vertex v1 = Vertex::v1;
        Vertex v2 = Vertex::v6;
        auto connected = vertex_connected(signed_distance, v1, v2);
        ASSERT_EQ(connected, false);
    }
}

TEST(GlobalTest, vertex_connected_cube_diagnal_connected)
{
    std::bitset<8> signed_distance{0b10111000};
    Vertex v1 = Vertex::v0;
    Vertex v2 = Vertex::v2;
    auto connected = vertex_connected(signed_distance, v1, v2);
    ASSERT_EQ(connected, true);
}

TEST(GlobalTest, vertex_connected_cube_diagnal_disconnected)
{
    {
        std::bitset<8> signed_distance{0b01111000};
        Vertex v1 = Vertex::v1;
        Vertex v2 = Vertex::v7;
        auto connected = vertex_connected(signed_distance, v1, v2);
        ASSERT_EQ(connected, false);
    }

    {
        std::bitset<8> signed_distance{0b10111110};
        Vertex v1 = Vertex::v0;
        Vertex v2 = Vertex::v6;
        auto connected = vertex_connected(signed_distance, v1, v2);
        ASSERT_EQ(connected, false);
    }
}

TEST(GlobalTest, has_value_bigger_than_zero_in_interval)
{
    double start = 0.0f, end = 1.0f;
    // test a bigger than 0, there is no value than 0
    {
        double a = 1.0f, b = 1.0f, c = -3.0f;
        auto result = has_value_bigger_than_zero_in_interval(a, b, c, start, end);
        ASSERT_EQ(result, false);
    }
    // test a bigger than 0, there are values bigger than 0
    // start is bigger than 0, but end not
    {
        double a = 1.0f, b = -3.0f, c = 1.0f;
        auto result = has_value_bigger_than_zero_in_interval(a, b, c, start, end);
        ASSERT_EQ(result, true);
    }
    // end is bigger than 0, but start not
    {
        double a = 1.0f, b = 1.0f, c = -1.0f;
        auto result = has_value_bigger_than_zero_in_interval(a, b, c, start, end);
        ASSERT_EQ(result, true);
    }
    // start and end are all bigger than 0
    {
        double a = 1.0f, b = 1.0f, c = 1.0f;
        auto result = has_value_bigger_than_zero_in_interval(a, b, c, start, end);
        ASSERT_EQ(result, true);
    }

    // test a smaller than 0
    // test no root
    {
        double a = -1.0f, b = -1.0f, c = -1.0f;
        auto result = has_value_bigger_than_zero_in_interval(a, b, c, start, end);
        ASSERT_EQ(result, false);
    }
    // test has root, but no value bigger than 0
    {
        double a = -1.0f, b = -3.0f, c = -1.0f;
        auto result = has_value_bigger_than_zero_in_interval(a, b, c, start, end);
        ASSERT_EQ(result, false);
    }
    // test has root, and has value bigger than 0
    {
        double a = -1.0f, b = 1.0f, c = -1.0f / 8.0f;
        auto result = has_value_bigger_than_zero_in_interval(a, b, c, start, end);
        ASSERT_EQ(result, true);
    }
}