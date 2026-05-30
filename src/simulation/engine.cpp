#include "engine.h"
#include "agent.h"
#include "processors/navigation.h"
#include "processors/avoidance.h"
#include "processors/movement.h"
#include "processors/walls.h"
#include "processors/formation.h"
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
static PathfindingContext pf_ctx;

void add_wall_tile(int x, int y)                     { wall_tiles.insert(encode_tile(x, y)); }
const std::unordered_set<uint64_t>& get_wall_tiles() { return wall_tiles; }

void set_target_in_rect(const Rectangle& rect, const Vector3& pos) {
    std::vector<int> members;
    for (int i = 0; i < agents.size(); ++i)
        if (CheckCollisionPointRec({ agents.positions[i].x, agents.positions[i].y }, rect))
            members.push_back(i);

    apply_formation(agents.positions, agents.targets,agents.nav_goal, members, pos);
}

void spawn_agent(const Vector3& pos) { agents.add(pos); }

void sim_tick(const float dt) {
    apply_navigation   (pf_ctx, agents.positions, agents.nav_goal, agents.targets, agents.vel, wall_tiles);
    apply_avoidance    (agents.positions, agents.targets, agents.vel, dt);
    apply_wall_collision(agents.positions, agents.vel, wall_tiles, dt);
    apply_movement      (agents.positions, agents.vel, dt);
}

int get_flow_field_count() { return (int)pf_ctx.cache.size(); }

size_t get_flow_field_bytes() {
    size_t bytes = 0;
    for (const auto& [key, field] : pf_ctx.cache)
        bytes += field.flow.capacity() * sizeof(Vector2);
    return bytes;
}

const std::vector<Vector3>& get_agent_positions() { return agents.positions; }
const std::vector<Vector3>& get_agent_targets()   { return agents.targets;   }
