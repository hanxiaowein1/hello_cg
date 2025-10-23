#include "mc33_lookup_table.h"
#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <bitset>
#include <queue>
#include <stdexcept>
#include "mc33_global_test.h"
#include <format>

bool g_mc33_table_initialized = false;
using mc33_table_type = decltype(MC33_TABLES);
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
                {{Vertex::v3, Vertex::v6}, {Vertex::v4, Vertex::v6}, {Vertex::v1, Vertex::v6}, {Vertex::v0, Vertex::v6}}, {0x1A2, 0x67B, 0x459},
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
            // TODO: interesting problem, when two diagonal points in the same face are connected by interpolation, so the correspond diagonal points in the cube is also connected by interpolation(this is definite, because in the range [0, 1], the boundary is connected, so the diagonal points in the cube is also connected), so in the future, I should simplify the list of connected points, such as simplify those connected points that can be duduced by other connection.
            {
                // {{Vertex::v3, Vertex::v5}, {Vertex::v0, Vertex::v2}, {Vertex::v2, Vertex::v4}, {Vertex::v1, Vertex::v7}},
                {{Vertex::v3, Vertex::v5}, {Vertex::v3, Vertex::v6}, {Vertex::v0, Vertex::v6}, {Vertex::v0, Vertex::v5}, {Vertex::v7, Vertex::v1}, {Vertex::v4, Vertex::v2}},
                {0xB6A, 0x2BA, 0x20B, 0xB08, 0x048, 0x409, 0x64A, 0x49A}
            },
            // case 10.2
            {
                // {{Vertex::v0, Vertex::v5}, {Vertex::v1, Vertex::v4}, {Vertex::v2, Vertex::v4}, {Vertex::v1, Vertex::v7}},
                {{Vertex::v3, Vertex::v6}, {Vertex::v1, Vertex::v4}, {Vertex::v7, Vertex::v1}, {Vertex::v4, Vertex::v2}, {Vertex::v3, Vertex::v5}, {Vertex::v0, Vertex::v6}},
                {0xCA2, 0xC9A, 0xC49, 0xC64, 0xCB6, 0xC8B, 0xC08, 0xC20}
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
            {
                {{Vertex::v3, Vertex::v4}, {Vertex::v4, Vertex::v6}, {Vertex::v2, Vertex::v4}}, {0x67B, 0x351, 0x385, 0x845}
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
                {{Vertex::v4, Vertex::v6}, {Vertex::v0, Vertex::v7}},
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

std::ostream& operator<<(std::ostream& os, const std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>& vertices)
{
    os << "{";
    for(const auto& vertice: vertices)
    {
        os << "{";
        for(const auto& vertex: vertice)
        {
            os << to_string(vertex) << ",";
        }
        os << "}, ";
    }
    os << "}";
    return os;
}

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

static std::unordered_map<Vertex, Vertex> VERTEX_UP_DOWN_FLIP = {
    {Vertex::v0, Vertex::v3},
    {Vertex::v1, Vertex::v2},
    {Vertex::v2, Vertex::v1},
    {Vertex::v3, Vertex::v0},
    {Vertex::v4, Vertex::v7},
    {Vertex::v5, Vertex::v6},
    {Vertex::v6, Vertex::v5},
    {Vertex::v7, Vertex::v4},
};
static std::unordered_map<Vertex, Vertex> VERTEX_LEFT_RIGHT_FLIP = {
    {Vertex::v0, Vertex::v1},
    {Vertex::v1, Vertex::v0},
    {Vertex::v2, Vertex::v3},
    {Vertex::v3, Vertex::v2},
    {Vertex::v4, Vertex::v5},
    {Vertex::v5, Vertex::v4},
    {Vertex::v6, Vertex::v7},
    {Vertex::v7, Vertex::v6},
};
static std::unordered_map<Vertex, Vertex> VERTEX_FRONT_BACK_FLIP = {
    {Vertex::v0, Vertex::v4},
    {Vertex::v1, Vertex::v5},
    {Vertex::v2, Vertex::v6},
    {Vertex::v3, Vertex::v7},
    {Vertex::v4, Vertex::v0},
    {Vertex::v5, Vertex::v1},
    {Vertex::v6, Vertex::v2},
    {Vertex::v7, Vertex::v3},
};


static std::unordered_map<Edge, Edge> EDGE_UP_DOWN_FLIP = {
    {Edge::e0, Edge::e2},
    {Edge::e1, Edge::e1},
    {Edge::e2, Edge::e0},
    {Edge::e3, Edge::e3},
    {Edge::e4, Edge::e6},
    {Edge::e5, Edge::e5},
    {Edge::e6, Edge::e4},
    {Edge::e7, Edge::e7},
    {Edge::e8, Edge::e11},
    {Edge::e9, Edge::e10},
    {Edge::e10, Edge::e9},
    {Edge::e11, Edge::e8},
    {Edge::ec, Edge::ec},
};
static std::unordered_map<Edge, Edge> EDGE_LEFT_RIGHT_FLIP = {
    {Edge::e0, Edge::e0},
    {Edge::e1, Edge::e3},
    {Edge::e2, Edge::e2},
    {Edge::e3, Edge::e1},
    {Edge::e4, Edge::e4},
    {Edge::e5, Edge::e7},
    {Edge::e6, Edge::e6},
    {Edge::e7, Edge::e5},
    {Edge::e8, Edge::e9},
    {Edge::e9, Edge::e8},
    {Edge::e10, Edge::e11},
    {Edge::e11, Edge::e10},
    {Edge::ec, Edge::ec},
};
static std::unordered_map<Edge, Edge> EDGE_FRONT_BACK_FLIP = {
    {Edge::e0, Edge::e4},
    {Edge::e1, Edge::e5},
    {Edge::e2, Edge::e6},
    {Edge::e3, Edge::e7},
    {Edge::e4, Edge::e0},
    {Edge::e5, Edge::e1},
    {Edge::e6, Edge::e2},
    {Edge::e7, Edge::e3},
    {Edge::e8, Edge::e8},
    {Edge::e9, Edge::e9},
    {Edge::e10, Edge::e10},
    {Edge::e11, Edge::e11},
    {Edge::ec, Edge::ec},
};


std::bitset<8> flip_vertex(const std::bitset<8>& distance_signs, FlipDir flip_dir)
{
    std::bitset<8> flipped_distance_signs;
    auto lambda = [&](std::unordered_map<Vertex, Vertex> vertex_flip_map){
        for(const auto& pair: vertex_flip_map)
        {
            flipped_distance_signs[static_cast<int>(pair.first)] = distance_signs[static_cast<int>(pair.second)];
        }
    };

    switch (flip_dir)
    {
    case FlipDir::ud:
        lambda(VERTEX_UP_DOWN_FLIP);
        break;
    case FlipDir::lr:
        lambda(VERTEX_LEFT_RIGHT_FLIP);
        break;
    case FlipDir::fb:
        lambda(VERTEX_FRONT_BACK_FLIP);
        break;
    default:
        break;
    }

    return flipped_distance_signs;
}

Edge flip_edge(const Edge& edge, const FlipDir& flip_dir)
{
    switch (flip_dir)
    {
    case FlipDir::ud:
        return EDGE_UP_DOWN_FLIP.at(edge);
    case FlipDir::lr:
        return EDGE_LEFT_RIGHT_FLIP.at(edge);
    case FlipDir::fb:
        return EDGE_FRONT_BACK_FLIP.at(edge);
    default:
        throw std::invalid_argument("unknown flip direction!");
    }
}

unsigned short flip_edge(const unsigned short& edges, const FlipDir& flip_dir)
{
    unsigned short flipped_edges = 0x0000;
    auto _edges = edges;
    // NOTE: take care! sequence need to be reverse too! You can take an example of vertex0 is negative and other vertices are positive, and flip it in left-right direction
    for(int i = 0; i < 3; i++)
    {
        _edges = _edges >> (i == 0 ? 0 : 4);
        auto edge = _edges & 0xF;
        Edge flipped_edge;
        flipped_edge = flip_edge(static_cast<Edge>(edge), flip_dir);
        flipped_edges = flipped_edges << 4;
        flipped_edges = flipped_edges | static_cast<unsigned short>(flipped_edge);
    }
    return flipped_edges;
}

std::vector<unsigned short> flip_edge(const std::vector<unsigned short>& edgess, const FlipDir& flip_dir)
{
    std::vector<unsigned short> flipped_edgess;
    for(const auto& edges: edgess)
    {
        auto flipped_edges = flip_edge(edges, flip_dir);
        flipped_edgess.emplace_back(flipped_edges);
    }
    return flipped_edgess;
}

/**
 * @brief reverse the sequence of the index of edge.
 * 
 * @param triangle: sample, 0x678
 * @return unsigned short: reversed triangle
 */
unsigned short reverse_triangle(const unsigned short& triangle)
{
    unsigned short reversed_triangle = 0x0000;
    for(int i = 0; i < 3; i++)
    {
        auto index = (triangle >> 4 * i) & 0x000F;
        reversed_triangle = (reversed_triangle << 4) | index;
    }
    return reversed_triangle;
}

bool equal(const unsigned short& edges1, const unsigned short& edges2)
{
    while(true)
    {
        Edge edge1_0 = static_cast<Edge>(edges1 & 0xF);
        int i = 0;
        while(true)
        {
            Edge edge2_0 = static_cast<Edge>(edges2 >> (4 * i) & 0xF);
            if(edge1_0 == edge2_0)
            {
                // compare the other two edge sequence(from back to front) and value
                Edge edge1_1 = static_cast<Edge>((edges1 >> 4) & 0xF);
                Edge edge1_2 = static_cast<Edge>((edges1 >> 8) & 0xF);
                Edge edge2_1, edge2_2;
                if(i == 0)
                {
                    edge2_1 = static_cast<Edge>((edges2 >> 4) & 0xF);
                    edge2_2 = static_cast<Edge>((edges2 >> 8) & 0xF);
                }
                if(i == 1)
                {
                    edge2_1 = static_cast<Edge>((edges2 >> 8) & 0xF);
                    edge2_2 = static_cast<Edge>(edges2 & 0xF);
                }
                if(i == 2)
                {
                    edge2_1 = static_cast<Edge>(edges2 & 0xF);
                    edge2_2 = static_cast<Edge>((edges2 >> 4) & 0xF);
                }
                if(edge1_1 == edge2_1 && edge1_2 == edge2_2)
                {
                    return true;
                }
                return false;
            }
            i++;
        }
        if(i == 3)
        {
            return false;
        }
    }
    return true;
}

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

void flip_tables(mc33_table_type& to_do_tables)
{
    // there are 8 situations,up-down flip(0, 1), left-right flip(0, 1), front-back flip(0, 1)
    std::vector<std::tuple<bool, bool, bool>> flip_situations{
        {false, false, false},
        {false, false, true},
        {false, true, false},
        {false, true, true},
        {true, false, false},
        {true, false, true},
        {true, true, false},
        {true, true, true}
    };
    auto lambda = [](const decltype(MC33_TABLES)::mapped_type& sub_cases, const FlipDir& flip_dir) -> decltype(MC33_TABLES)::mapped_type {
        decltype(MC33_TABLES)::mapped_type flipped_sub_cases;
        for(const auto& [connected_vertices, triangles]: sub_cases)
        {
            std::remove_const_t<decltype(connected_vertices)> flipped_connected_vertices;
            // TODO: figure out why this won't work
            /*decltype(sub_cases)::key_type flipped_connected_vertices;*/
            std::remove_const_t<decltype(triangles)> flipped_triangles;
            flipped_triangles = flip_edge(triangles, flip_dir);
            for(auto connected_vertice: connected_vertices)
            {
                decltype(connected_vertice) flipped_connected_vertice;
                for(auto vertex: connected_vertice)
                {
                    Vertex flipped_vertex;
                    // flip vertex
                    switch (flip_dir)
                    {
                    case FlipDir::ud:
                        flipped_vertex = VERTEX_UP_DOWN_FLIP.at(vertex);
                        break;
                    case FlipDir::lr:
                        flipped_vertex = VERTEX_LEFT_RIGHT_FLIP.at(vertex);
                        break;
                    case FlipDir::fb:
                        flipped_vertex = VERTEX_FRONT_BACK_FLIP.at(vertex);
                        break;
                    default:
                        break;
                    }
                    flipped_connected_vertice.emplace(flipped_vertex);
                }
                // NOTE: I staggered in this line for 1 hours, it's not because the problem of the type or hash function, it's the reason that once define the flipped_connected_vertices variable by decltype(connected_vertices), which is const and cannot be modified...
                flipped_connected_vertices.emplace(flipped_connected_vertice);
            }
            flipped_sub_cases.emplace(std::make_pair(flipped_connected_vertices, flipped_triangles));
        }
        return flipped_sub_cases;
    };
    decltype(MC33_TABLES) flipped_tables;
    for(const auto& [distance_signs, sub_cases]: to_do_tables)
    {
        for(const auto& flip_situation: flip_situations)
        {
            auto flipped_distance_signs = distance_signs;
            auto flipped_sub_cases = sub_cases;
            auto [ud_flip, lr_flip, fb_flip] = flip_situation;
            if(ud_flip)
            {
                // do up-down flip
                flipped_distance_signs = flip_vertex(flipped_distance_signs, FlipDir::ud);
                flipped_sub_cases = lambda(flipped_sub_cases, FlipDir::ud);
            }
            if(lr_flip)
            {
                // do left-right flip
                flipped_distance_signs = flip_vertex(flipped_distance_signs, FlipDir::lr);
                flipped_sub_cases = lambda(flipped_sub_cases, FlipDir::lr);
            }
            if(fb_flip)
            {
                // do front-back flip
                flipped_distance_signs = flip_vertex(flipped_distance_signs, FlipDir::fb);
                flipped_sub_cases = lambda(flipped_sub_cases, FlipDir::fb);
            }
            // insert new cases into mc33 table
            if(flipped_tables.contains(flipped_distance_signs))
            {
                flipped_tables[flipped_distance_signs].insert(flipped_sub_cases.begin(), flipped_sub_cases.end());
            }
            else
            {
                flipped_tables[flipped_distance_signs] = flipped_sub_cases;
            }
        }
    }
    // merge flipped tables and to_do_tables
    for (const auto& [distance_signs, sub_cases] : flipped_tables)
    {
        if (to_do_tables.contains(distance_signs))
        {
            to_do_tables[distance_signs].insert(sub_cases.begin(), sub_cases.end());
        }
        else
        {
            to_do_tables[distance_signs] = sub_cases;
        }
    }
}

void invert_tables(mc33_table_type& to_do_tables)
{
    decltype(MC33_TABLES) inverted_tables;
    for(const auto& [distance_signs, sub_cases]: to_do_tables)
    {
        auto inverted_distances_signs = distance_signs;
        inverted_distances_signs.flip();
        decltype(MC33_TABLES)::mapped_type inverted_sub_cases;
        for(const auto& [connected_vertices, triangles]: sub_cases)
        {
            std::remove_const_t<decltype(triangles)> inverted_triangles;
            for(const auto& triangle: triangles)
            {
                auto inverted_triangle = reverse_triangle(triangle);
                inverted_triangles.emplace_back(inverted_triangle);
            }
            inverted_sub_cases[connected_vertices] = inverted_triangles;
        }
        inverted_tables[inverted_distances_signs] = inverted_sub_cases;
    }
    // merge inverted_tables and to_do_tables
    for(const auto& [distance_signs, sub_cases]: inverted_tables)
    {
        if(to_do_tables.contains(distance_signs))
        {
            to_do_tables[distance_signs].insert(sub_cases.begin(), sub_cases.end());
        }
        else
        {
            to_do_tables[distance_signs] = sub_cases;
        }
    }
}

void init_tables()
{
    if(g_mc33_table_initialized == true)
    {
        return;
    }
    g_mc33_table_initialized = true;
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

    flip_tables(MC33_TABLES);

    // TODO: pure rotate is not enough, the sign distance may be already 256, but the inner mesh need to be expanded

    // invert distance signs
    // TODO: there must be a bug, when invert signs, the edge direction also need to be altered, because the face direction has been changed
    // std::unordered_map<
    //     std::bitset<8>,
    //     std::unordered_map<
    //         std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>,
    //         std::vector<unsigned short>,
    //         boost::hash<std::unordered_set<std::unordered_set<Vertex>, boost::hash<std::unordered_set<Vertex>>>>
    //     >
    // > invert_table;
    // for(auto [key, value]: MC33_TABLES)
    // {
    //     auto origin = key;
    //     origin.flip();
    //     invert_table.emplace(std::make_pair(origin, value));
    // }

    // MC33_TABLES.insert(invert_table.begin(), invert_table.end());

    invert_tables(MC33_TABLES);

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

namespace
{
class InitMC33Table
{
public:
    InitMC33Table()
    {
        ::init_tables();
    }
    ~InitMC33Table() = default;
};
}

static InitMC33Table g_init_mc33_table;

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

TEST(GlobalTest, flip_vertex_map)
{
    // function to test if the flipped vertex of a flipped vertex is vertex
    auto lambda = [](const decltype(VERTEX_FRONT_BACK_FLIP)& flip_table) -> bool {
        for(const auto& [vertex, flipped_vertex]: flip_table)
        {
            if(flip_table.contains(flipped_vertex))
            {
                if(flip_table.at(flipped_vertex) != vertex)
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    };

    // up-down
    ASSERT_EQ(true, lambda(VERTEX_UP_DOWN_FLIP));
    // left-right
    ASSERT_EQ(true, lambda(VERTEX_LEFT_RIGHT_FLIP));
    // front-back
    ASSERT_EQ(true, lambda(VERTEX_FRONT_BACK_FLIP));
}

TEST(GlobalTest, flip_vertex)
{
    std::bitset<8> distance_signs{0b10110010};

    // up-down flip test
    {
        auto flipped_distance_signs = flip_vertex(distance_signs, FlipDir::ud);
        ASSERT_EQ(flipped_distance_signs, std::bitset<8>(0b11010100));
    }
    // left-right flip test
    {
        auto flipped_distance_signs = flip_vertex(distance_signs, FlipDir::lr);
        ASSERT_EQ(flipped_distance_signs, std::bitset<8>(0b01110001));
    }
    // front-back flip test
    {
        auto flipped_distance_signs = flip_vertex(distance_signs, FlipDir::fb);
        ASSERT_EQ(flipped_distance_signs, std::bitset<8>(0b00101011));
    }
}

TEST(GlobalTest, flip_edge_map)
{
    // the flip of the flipped edge is edge
    auto lambda = [](const decltype(EDGE_LEFT_RIGHT_FLIP) flip_table) -> bool {
        for(const auto& [edge, flipped_edge]: flip_table)
        {
            if(flip_table.contains(flipped_edge))
            {
                if(flip_table.at(flipped_edge) != edge)
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    };

    // up-down
    ASSERT_EQ(true, lambda(EDGE_UP_DOWN_FLIP));
    // left-right
    ASSERT_EQ(true, lambda(EDGE_LEFT_RIGHT_FLIP));
    // front-back
    ASSERT_EQ(true, lambda(EDGE_FRONT_BACK_FLIP));
}

// TODO: I can use a macro to better name the unit test, like DEFINE_TEST(func, type1, type2, xxx)
// NOTE: next time if I want to do some improvement, I should not move to it directly because my main mission is not finished yet. I should mark it as TODO and after I finish the main mission, I can take a review and do it later when I have free time(Or stagger in the main mission and want to change a topic to relax, after all, study new knowledge is really relaxing).
TEST(GlobalTest, flip_edge_const_Edge_const_FlipDir)
{
    // test up-down
    {
        Edge edge = Edge::e0;
        FlipDir flip_dir = FlipDir::ud;
        auto flipped_edge = flip_edge(edge, flip_dir);
        ASSERT_EQ(Edge::e2, flipped_edge);
    }
    // test left-right
    {
        Edge edge = Edge::e0;
        FlipDir flip_dir = FlipDir::lr;
        auto flipped_edge = flip_edge(edge, flip_dir);
        ASSERT_EQ(Edge::e0, flipped_edge);
    }
    // test front-back
    {
        Edge edge = Edge::e0;
        FlipDir flip_dir = FlipDir::fb;
        auto flipped_edge = flip_edge(edge, flip_dir);
        ASSERT_EQ(Edge::e4, flipped_edge);
    }
}

// TODO: I have a new idea, create a template or macro define, pass function pointer and parameter to it, it can directly generate unit test by the funciton name and parameter type, like DEFINE_UNIT_TEST(func1, type1, type2, ...)
TEST(GlobalTest, flip_edge_const_unsigned_short_const_FlipDir)
{
    // up-down
    {
        unsigned short edges = 0x0038;
        FlipDir flip_dir = FlipDir::ud;
        auto flipped_edges = flip_edge(edges, flip_dir);
        ASSERT_EQ(flipped_edges, 0x0B32);
    }
    // left-right
    {
        unsigned short edges = 0x0038;
        FlipDir flip_dir = FlipDir::lr;
        auto flipped_edges = flip_edge(edges, flip_dir);
        ASSERT_EQ(flipped_edges, 0x0910);
    }
    // front-back
    {
        unsigned short edges = 0x0038;
        FlipDir flip_dir = FlipDir::fb;
        auto flipped_edges = flip_edge(edges, flip_dir);
        ASSERT_EQ(flipped_edges, 0x0874);
    }
}

TEST(GlobalTest, flip_edge_const_vector_unsigned_short_const_FlipDir)
{
    // up-down
    {
        std::vector<unsigned short> edgess{0x0038, 0x0C49};
        FlipDir flip_dir = FlipDir::ud;
        auto flipped_edgess = flip_edge(edgess, flip_dir);
        decltype(flipped_edgess) expected_flipped_edgess{0x0B32, 0x0A6C};
        ASSERT_EQ(edgess.size(), flipped_edgess.size());
        for(int i = 0; i < edgess.size(); i++)
        {
            auto flipped_edges = flipped_edgess[i];
            auto expected_flipped_edges = expected_flipped_edgess[i];
            ASSERT_EQ(flipped_edges, expected_flipped_edges) << std::format("the {} flipped edges is not same with the expected flipped edges", i) << std::endl;
        }
    }
    // left-right
    {
        std::vector<unsigned short> edgess{0x0038, 0x0C49};
        FlipDir flip_dir = FlipDir::lr;
        auto flipped_edgess = flip_edge(edgess, flip_dir);
        decltype(flipped_edgess) expected_flipped_edgess{ 0x0910, 0x084C};
        ASSERT_EQ(edgess.size(), flipped_edgess.size());
        for(int i = 0; i < edgess.size(); i++)
        {
            auto flipped_edges = flipped_edgess[i];
            auto expected_flipped_edges = expected_flipped_edgess[i];
            ASSERT_EQ(flipped_edges, expected_flipped_edges) << std::format("the {} flipped edges is not same with the expected flipped edges", i) << std::endl;
        }
    }
    // front-back
    {
        std::vector<unsigned short> edgess{0x0038, 0x0C49};
        FlipDir flip_dir = FlipDir::fb;
        auto flipped_edgess = flip_edge(edgess, flip_dir);
        decltype(flipped_edgess) expected_flipped_edgess{ 0x0874, 0x090C};
        ASSERT_EQ(edgess.size(), flipped_edgess.size());
        for(int i = 0; i < edgess.size(); i++)
        {
            auto flipped_edges = flipped_edgess[i];
            auto expected_flipped_edges = expected_flipped_edgess[i];
            ASSERT_EQ(flipped_edges, expected_flipped_edges) << std::format("the {} flipped edges is not same with the expected flipped edges", i) << std::endl;
        }
    }
}

TEST(GlobalTest, flip_tables)
{
    decltype(MC33_TABLES) tables;
    tables[std::bitset<8>(0b10101111)] = {
        {
            {{Vertex::v5, Vertex::v7}},
            {0x487, 0x56A},
        }
    };
    flip_tables(tables);
    //ASSERT_EQ(tables.size(), 8);
    decltype(MC33_TABLES) expected_tables = {
        {
            0b10101111,
            {
                {
                    {{Vertex::v5, Vertex::v7}},
                    {0x487, 0x56A},
                }
            }
        },
        {
            0b11111010,
            {
                {
                    {{Vertex::v1, Vertex::v3}},
                    {0x0380, 0x0A21},
                }
            }
        },
        {
            0b01011111,
            {
                {
                    {{Vertex::v4, Vertex::v6}},
                    {0x594, 0x0B67},
                }
            }
        },
        {
            0b11110101,
            {
                {
                    {{Vertex::v0, Vertex::v2}},
                    {0x0091, 0x032B},
                }
            }
        },
        // NOTE: interesting, when there are only one negative vertex, the flipped table's size is 8, when there are two of them, the size turns to 4, because half of them are duplicate
        // {
        //     0b01011111,
        //     {
        //         {
        //             {{Vertex::v6, Vertex::v4}},
        //             {0x07B6, 0x0945},
        //         }
        //     }
        // },
        // {
        //     0b11110101,
        //     {
        //         {
        //             {{Vertex::v2, Vertex::v0}},
        //             {0x02B3, 0x0109},
        //         }
        //     }
        // },
        // {
        //     0b10101111,
        //     {
        //         {
        //             {{Vertex::v7, Vertex::v5}},
        //             {0x06A5, 0x0847},
        //         }
        //     }
        // },
        // {
        //     0b11111010,
        //     {
        //         {
        //             {{Vertex::v1, Vertex::v3}},
        //             {0x01A2, 0x0803},
        //         }
        //     }
        // },
    };
    // check if the size is same
    ASSERT_EQ(tables.size(), expected_tables.size());
    for(const auto& [distance_signs, sub_cases]: tables)
    {
        if(!expected_tables.contains(distance_signs))
        {
            FAIL() << std::format("there no distance sign {} in expected result", distance_signs.to_string());
        }
        else
        {
            // check if sub cases size is the same
            ASSERT_EQ(expected_tables[distance_signs].size(), sub_cases.size());
            // check if the inner content are equal
            for(const auto& [connected_vertices, triangles]: sub_cases)
            {
                if(!expected_tables[distance_signs].contains(connected_vertices))
                {
                    FAIL() << "there are no same connected vertices in expected tables which are:" << connected_vertices << std::endl;
                }
                else
                {
                    // check if the inner triangles are equal
                    auto expected_triangles = expected_tables[distance_signs][connected_vertices];
                    ASSERT_EQ(triangles.size(), expected_triangles.size());
                    for(int i = 0; i < triangles.size(); i++)
                    {
                        ASSERT_EQ(true, equal(triangles[i], expected_triangles[i])) << std::format("in distance sign {}, connected vertices: ", distance_signs.to_string()) << connected_vertices << std::format(", triangles is different! Real triangle is {:#0x}, expected one is {:#0x}", triangles[i], expected_triangles[i]);
                    }
                }
            }
        }
    }
}

TEST(GlobalTest, invert_tables)
{
    decltype(MC33_TABLES) tables;
    tables[std::bitset<8>(0b10101111)] = {
        {
            {{Vertex::v5, Vertex::v7}},
            {0x487, 0x56A},
        }
    };
    invert_tables(tables);
    ASSERT_EQ(tables.size(), 2);
    decltype(MC33_TABLES) expected_tables = {
        {
            0b10101111,
            {
                {
                    {{Vertex::v5, Vertex::v7}},
                    {0x487, 0x56A},
                }
            }
        },
        {
            0b01010000,
            {
                {
                    {{Vertex::v5, Vertex::v7}},
                    {0x784, 0xA65},
                }
            }
        },
    };
    ASSERT_EQ(tables.size(), expected_tables.size());
    // TODO: duplicate code, need to be simplifed in the future
    for(const auto& [distance_signs, sub_cases]: tables)
    {
        if(!expected_tables.contains(distance_signs))
        {
            FAIL() << std::format("there no distance sign {} in expected result", distance_signs.to_string());
        }
        else
        {
            // check if sub cases size is the same
            ASSERT_EQ(expected_tables[distance_signs].size(), sub_cases.size());
            // check if the inner content are equal
            for(const auto& [connected_vertices, triangles]: sub_cases)
            {
                if(!expected_tables[distance_signs].contains(connected_vertices))
                {
                    FAIL() << "there are no same connected vertices in expected tables which are:" << connected_vertices << std::endl;
                }
                else
                {
                    // check if the inner triangles are equal
                    auto expected_triangles = expected_tables[distance_signs][connected_vertices];
                    ASSERT_EQ(triangles.size(), expected_triangles.size());
                    for(int i = 0; i < triangles.size(); i++)
                    {
                        ASSERT_EQ(true, equal(triangles[i], expected_triangles[i])) << std::format("in distance sign {}, connected vertices: ", distance_signs.to_string()) << connected_vertices << std::format(", triangles is different! Real triangle is {:#0x}, expected one is {:#0x}", triangles[i], expected_triangles[i]);
                    }
                }
            }
        }
    }
}

TEST(GlobalTest, equal_triangle)
{
    ASSERT_EQ(true, equal(0x123, 0x231));
    ASSERT_EQ(true, equal(0x123, 0x312));

    ASSERT_EQ(false, equal(0x123, 0x132));
    ASSERT_EQ(false, equal(0x123, 0x213));

}