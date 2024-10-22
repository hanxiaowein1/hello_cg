#ifndef _CHARLES_GEOMETRY_H_
#define _CHARLES_GEOMETRY_H_

#include "basic_type.h"

template<typename Type>
Vector3<Type> crossProduct(const Vector3<Type>& a, const Vector3<Type>& b);

template<typename Type>
Type dotProduct(const Vector3<Type>& a, const Vector3<Type>& b);

template<typename Type>
bool ray_triangle_intersect(
    const Vector3<Type>& v0, const Vector3<Type>& v1, const Vector3<Type>& v2,
    const Vector3<Type>& orig, const Vector3<Type>& dir, Type& tnear, Type& u, Type& v
);

#endif