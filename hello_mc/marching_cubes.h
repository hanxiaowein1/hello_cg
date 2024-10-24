#ifndef __CHARLES_MARCHING_CUBES_H__
#define __CHARLES_MARCHING_CUBES_H__

#include <vector>
#include <string>
#include "Eigen/Dense"
#include "charles_mc_types.h"

Eigen::Vector3d get_vertex(
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance,
    bool on_x, bool on_y, bool on_z,
    Vertex first_point, Vertex second_point
);
void write_obj(std::string filename, const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3i>& triangles);
Eigen::Vector3d get_vertex_by_edge(
    Edge edge,
    const std::vector<Eigen::Vector3d>& coors,
    const std::vector<double>& signed_distance
);

#endif