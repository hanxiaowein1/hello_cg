#include "cgl.h"

template<typename Type>
Vector3<Type> crossProduct(const Vector3<Type>& a, const Vector3<Type>& b)
{
    return Vector3<Type>{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

template<typename Type>
Type dotProduct(const Vector3<Type>& a, const Vector3<Type>& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template<typename Type>
bool ray_triangle_intersect(
    const Vector3<Type>& v0, const Vector3<Type>& v1, const Vector3<Type>& v2,
    const Vector3<Type>& orig, const Vector3<Type>& dir, Type& tnear, Type& u, Type& v
)
{
    auto p0 = v0;
    auto p1 = v1;
    auto p2 = v2;
    auto e1 = p1 - p0;
    auto e2 = p2 - p0;
    auto s = orig - p0;
    auto s1 = crossProduct(dir, e2);
    auto s2 = crossProduct(s, e1);
    auto s1_e1_dot = dotProduct(s1, e1);
    tnear = (1.0f / s1_e1_dot) * dotProduct(s2, e2);
    u = (1.0f / s1_e1_dot) * dotProduct(s1, s);
    v = (1.0f / s1_e1_dot) * dotProduct(s2, dir);
    if(u >= 0.0f && v >=0.0f && (1.0f - u - v) >=0.0f && tnear >= 0.0f){
        return true;
    }
    return false;
}