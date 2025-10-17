#include "charles_mc33_type.h"

std::string to_string(const Vertex& vertex)
{
    switch (vertex)
    {
    case Vertex::v0:
        return "v0";
    case Vertex::v1:
        return "v1";
    case Vertex::v2:
        return "v2";
    case Vertex::v3:
        return "v3";
    case Vertex::v4:
        return "v4";
    case Vertex::v5:
        return "v5";
    case Vertex::v6:
        return "v6";
    case Vertex::v7:
        return "v7";
    case Vertex::vc1:
        return "vc1";
    case Vertex::vc2:
        return "vc2";
    default:
        return "unknown";
    }
}