#include "engine.h"
#include "agent.h"
#include "processors/steering.h"
#include "processors/avoidance.h"
#include "processors/movement.h"

static std::vector<Agent> make_agents() {
    std::vector<Agent> out;
    constexpr int cols = 10, rows = 5;
    constexpr float spacing = 60.0f;
    constexpr float ox = 80.0f, oy = 80.0f;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            out.push_back({ { ox + c * spacing, oy + r * spacing, 0.0f } });
    return out;
}

static std::vector<Agent> agents = make_agents();

static SteeringConfig  steering_cfg;
static AvoidanceConfig avoidance_cfg;

SteeringConfig&  get_steering_config()  { return steering_cfg; }
AvoidanceConfig& get_avoidance_config() { return avoidance_cfg; }

void set_target_in_rect(const Rectangle& rect, const Vector3& pos) {
    for (auto& a : agents) {
        if (CheckCollisionPointRec({ a.position.x, a.position.y }, rect)) {
            a.target = pos;
            a.has_target = true;
        }
    }
}

void sim_tick(const float dt) {
    std::vector<Vector3> vel(agents.size(), { 0.0f, 0.0f, 0.0f });

    apply_steering(agents, vel, steering_cfg);
    apply_avoidance(agents, vel, avoidance_cfg);
    apply_movement(agents, vel, dt);
    apply_separation(agents, avoidance_cfg);
}

const std::vector<Vector3> get_agent_positions() {
    std::vector<Vector3> out;
    out.reserve(agents.size());
    for (const auto& a : agents)
        out.push_back(a.position);
    return out;
}

const std::vector<Vector3> get_agent_targets() {
    std::vector<Vector3> out;
    for (const auto& a : agents)
        if (a.has_target)
            out.push_back(a.target);
    return out;
}
