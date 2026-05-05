#pragma once
#include "raylib.h"
#include "walls.h"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <vector>
#include <cstdint>

struct FlowField {
    using PQEntry = std::pair<float, uint64_t>;
    std::unordered_map<uint64_t, float>   dist;
    std::unordered_map<uint64_t, Vector2> dir;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> frontier;
    bool exhausted = false;
};

void apply_pathfinding(
    const std::vector<Vector3>&         positions,
    const std::vector<Vector3>&         targets,
    const std::vector<float>&           arrived,
    const std::unordered_set<uint64_t>& wall_tiles);

void pathfinding_invalidate();
const std::vector<Vector3>&                      get_waypoints();
const std::unordered_map<uint64_t, FlowField>&   get_flow_fields();
