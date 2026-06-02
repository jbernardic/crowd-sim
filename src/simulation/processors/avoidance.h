#pragma once
#include "raylib.h"
#include <vector>

// Persistent per-agent avoidance state. Owned by the caller and passed in so no
// state is global. `settled` is latched: it is set once an agent reaches its slot
// and is NOT cleared just because the agent is later nudged off the slot -- only a
// new order (its target changing) clears it. `last_target` tracks the target seen
// last tick so that target change can be detected.
struct AvoidanceContext {
    std::vector<char>    settled;
    std::vector<Vector3> last_target;
};

void apply_avoidance(
    AvoidanceContext&           ctx,
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    float                       dt);
