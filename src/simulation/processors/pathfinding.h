#pragma once
#include "raylib.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <cstddef>

// One cached flow field: per-cell unit vectors pointing toward a goal.
struct FlowField {
    int gw = 0, gh = 0;
    std::vector<Vector2> flow;   // per cell; {0,0} = goal or unreachable
};

// Holds the flow-field cache and the inputs it was built for. Owned by the
// caller and passed into apply_pathfinding so no pathfinding state is global.
struct PathfindingContext {
    std::unordered_map<uint64_t, FlowField> cache;
    uint64_t wall_xor   = 0;
    size_t   wall_count = (size_t)-1;
    float    built_ts   = -1.0f;
    int      built_w    = -1;
    int      built_h    = -1;
};

// `nav_goal` is the point each agent paths toward (a per-formation shared anchor,
// not the agent's individual slot). Routing to a shared anchor means one cached
// flow field per formation instead of one per agent. `destinations` is still the
// agent's own slot; once within the blob the agent steers straight to it.
void apply_pathfinding(
    PathfindingContext&                 ctx,
    const std::vector<Vector3>&         positions,
    std::vector<Vector3>&               targets,
    const std::vector<Vector3>&         destinations,
    const std::vector<Vector3>&         nav_goal,
    const std::unordered_set<uint64_t>& wall_tiles);