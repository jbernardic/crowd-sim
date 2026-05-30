#pragma once
#include "raylib.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <cstddef>

// One cached flow field: per-cell unit vectors pointing toward a goal. The field
// covers an arbitrary rectangle of the (unbounded) world grid: local cell (0,0)
// sits at world cell (ox, oy), so the field is not tied to the window.
struct FlowField {
    int gw = 0, gh = 0;          // local grid dimensions
    int ox = 0, oy = 0;          // world cell of local cell (0,0)
    std::vector<Vector2> flow;   // per cell; {0,0} = goal or unreachable
};

// Holds the flow-field cache and the inputs it was built for. Owned by the
// caller and passed into apply_navigation so no pathfinding state is global.
// The whole cache is dropped only when the walls or tile size change (those
// invalidate every field at once). Region drift as the agents roam does not
// invalidate fields; individual fields are rebuilt on demand when an agent
// leaves their window (see apply_navigation).
struct PathfindingContext {
    std::unordered_map<uint64_t, FlowField> cache;
    uint64_t wall_xor   = 0;
    size_t   wall_count = (size_t)-1;
    float    built_ts   = -1.0f;
};

// Drives every agent's velocity toward its goal in a single step (flow-field
// pathfinding + steering combined).
//
// Each agent carries two goals:
//   nav_goal[i] - the GLOBAL goal: a per-formation shared anchor. Long-range
//                 routing follows one cached flow field per anchor, so a
//                 formation builds one field instead of one per slot.
//   targets[i]  - the LOCAL goal: the agent's own slot. Once inside the blob the
//                 agent leaves the field and steers straight to it.
//
// The per-tick steering point ("carrot") is purely internal here. Velocity eases
// off within the slot radius so agents settle gently onto their slots.
void apply_navigation(
    PathfindingContext&                 ctx,
    const std::vector<Vector3>&         positions,
    const std::vector<Vector3>&         nav_goal,
    const std::vector<Vector3>&         targets,
    std::vector<Vector3>&               vel,
    const std::unordered_set<uint64_t>& wall_tiles);
