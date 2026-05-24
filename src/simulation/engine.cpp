#include "engine.h"
#include "agent.h"
#include "processors/steering.h"
#include "processors/avoidance.h"
#include "processors/separation.h"
#include "processors/movement.h"
#include "processors/arrival.h"
#include "processors/walls.h"
#include "processors/pathfinding.h"
#include <unordered_set>

static AgentData make_agents() {
    AgentData data;
    constexpr int cols = 10, rows = 5;
    constexpr float spacing = 60.0f;
    constexpr float ox = 80.0f, oy = 80.0f;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            data.add({ ox + c * spacing, oy + r * spacing, 0.0f });
    return data;
}

static AgentData agents = make_agents();
static std::unordered_set<uint64_t> wall_tiles;

void add_wall_tile(int x, int y)                     { wall_tiles.insert(encode_tile(x, y)); }
const std::unordered_set<uint64_t>& get_wall_tiles() { return wall_tiles; }

void set_target_in_rect(const Rectangle& rect, const Vector3& pos) {
    for (int i = 0; i < agents.size(); ++i) {
        if (CheckCollisionPointRec({ agents.positions[i].x, agents.positions[i].y }, rect)) {
            agents.targets[i] = pos;
            agents.destinations[i] = pos;
            agents.arrived[i] = 0.0f;
        }
    }
}

void sim_tick(const float dt) {
    apply_pathfinding (agents.positions, agents.targets, agents.destinations, wall_tiles);
    apply_steering     (agents.positions, agents.targets, agents.vel);
    apply_avoidance    (agents.positions, agents.vel, dt);
    apply_arrival      (agents.positions, agents.targets, agents.vel, agents.arrived, dt);
    apply_separation   (agents.positions, agents.vel, dt);
    apply_wall_collision(agents.positions, agents.vel, wall_tiles, dt);
    apply_movement      (agents.positions, agents.vel, dt);
}

const std::vector<Vector3>& get_agent_positions() { return agents.positions; }
const std::vector<Vector3>& get_agent_targets()   { return agents.targets;   }
const std::vector<float>&   get_agent_arrived()   { return agents.arrived;   }
