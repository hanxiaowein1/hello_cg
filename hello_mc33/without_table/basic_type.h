#ifndef _CHARLES_BASIC_TYPE_H_
#define _CHARLES_BASIC_TYPE_H_

template<typename CoorType>
class Point3
{
public:
    CoorType x;
    CoorType y;
    CoorType z;
};

template<typename Type>
class Vector3
{
public:
    Type x;
    Type y;
    Type z;
};

enum class Vertex {v0, v1, v2, v3, v4, v5, v6, v7};

enum class Edge {e0, e1, e2, e3, e4, e5, e6, e7, e8, e9, ea, eb};

enum class Face {f0, f1, f2, f3, f4, f5};

#endif