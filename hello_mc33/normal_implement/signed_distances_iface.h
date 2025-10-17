#ifndef __CHARLES_SIGNED_DISTANCES_IFACE_H__
#define __CHARLES_SIGNED_DISTANCES_IFACE_H__

#include <string>
#include "Eigen/Dense"

void generate_signed_distance(const int& nx, const int& ny, const int &nz, const std::string& mesh_path, const std::string& save_path);
Eigen::VectorXd get_signed_distance(int &nx, int &ny, int &nz, const std::string& sd_path);

#endif