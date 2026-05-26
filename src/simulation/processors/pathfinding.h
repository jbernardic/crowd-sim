#pragma once
#include "raylib.h"
#include <unordered_set>
#include <vector>
#include <cstdint>

void apply_pathfinding(
    const std::vector<Vector3>&         positions,
    std::vector<Vector3>&               targets,
    const std::vector<Vector3>&         destinations,
    const std::unordered_set<uint64_t>& wall_tiles);