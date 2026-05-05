#pragma once
#include "raylib.h"
#include <cstdint>
#include <unordered_set>
#include <vector>

inline uint64_t encode_tile(int x, int y) {
    return ((uint64_t)(uint32_t)x << 32) | (uint32_t)y;
}

inline void decode_tile(uint64_t key, int& x, int& y) {
    x = (int)(uint32_t)(key >> 32);
    y = (int)(uint32_t)(key & 0xFFFFFFFF);
}

void apply_wall_collision(
    const std::vector<Vector3>&         positions,
    std::vector<Vector3>&               vel,
    const std::unordered_set<uint64_t>& tiles,
    float                               dt);
