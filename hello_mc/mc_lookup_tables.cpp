#include "mc_lookup_tables.h"

// key: v8v7v6v5v4v3v2v1v0(0 is negative, 1 is positive, remember: bitset stores reversely)
std::unordered_map<std::bitset<8>, std::vector<unsigned short int>> MC_Tables = {
    {0b11111111, {}},

    /**
     * @brief case 1
     * 
     */
    {0b11111110, {0x038}},  // v0
    {0b11111101, {0x019}},  // v1
    {0b11111011, {0x12A}},  // v2
    {0b11110111, {0x23B}},  // v3
    {0b11101111, {0x478}},  // v4
    {0b11011111, {0x459}},  // v5
    {0b10111111, {0x56A}},  // v6
    {0b01111111, {0x67B}},  // v7

    /**
     * @brief case 2
     * 
     */
    {0b11111100, {0x381, 0x189}},
    {0b11111001, {0x2A0, 0x09A}},
    {0b11110011, {0x3BA, 0x13A}},
    {0b11100111, {0x2B0, 0xB80}},
    {0b11001111, {0x785, 0x589}},
    {0b10011111, {0x6A9, 0x649}},
    {0b00111111, {0x5AB, 0x5B7}},
    {0b01101111, {0x6B8, 0x684}},
    {0b11101110, {0x037, 0x074}},
    {0b11011101, {0x014, 0x145}},
    {0b10111011, {0x126, 0x165}},
    {0b01110111, {0x236, 0x367}},

    /**
     * @brief case 8
     * 
     */
    {0b11001100, {0x137, 0x175}},  // face 0
    {0b00110011, {0x137, 0x175}},  // face 2
    {0b10011001, {0x026, 0x046}},  // face 1
    {0b01100110, {0x026, 0x046}},  // face 3
    {0b11110000, {0x9AB, 0x9B8}},  // face 4
    {0b00001111, {0x9AB, 0x9B8}},  // face 5
};