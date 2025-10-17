#ifndef __CHARLES_MC33_TYPES_H__
#define __CHARLES_MC33_TYPES_H__
#include <string>

enum class Vertex {v0, v1, v2, v3, v4, v5, v6, v7, vc1, vc2};

enum class Edge {e0, e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, ec};

enum class Face {f0, f1, f2, f3, f4, f5};

enum class Axis {x, y, z};

// flip direction: up-down, left-right, front-back
enum class FlipDir {ud, lr, fb};

std::string to_string(const Vertex& vertex);


//namespace boost {
//template <>
//struct hash<Vertex> {
//    std::size_t operator()(Vertex e) const noexcept {
//        return std::hash<int>{}(static_cast<int>(e));
//    }
//};
//}  // n

#endif