#ifndef __CHARLES_MC33_H__
#define __CHARLES_MC33_H__

#include <string>
#include <tuple>

void generate_mesh(const std::string& sd_path, const std::string save_path);
std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<int>>> generate_mesh(const std::string& sd_path);
#endif