#ifndef __CHARLES_MC_LOOKUP_TABLES__
#define __CHARLES_MC_LOOKUP_TABLES__

#include <unordered_set>
#include <unordered_map>
#include <bitset>
#include <vector>
#include <boost/functional/hash.hpp>

#include "charles_mc_types.h"

extern std::unordered_map<std::bitset<8>, std::vector<unsigned short int>> MC_TABLES;

void invert_tables();

void init_tables();

void print_mc_tables();

#endif