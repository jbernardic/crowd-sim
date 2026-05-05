#include "engine.h"
#include "agent.h"
#include "processors/steering.h"
#include "processors/avoidance.h"
#include "processors/movement.h"
#include "processors/arrival.h"

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

static SteeringConfig  steering_cfg;
static AvoidanceConfig avoidance_cfg;
static ArrivalConfig   arrival_cfg;

SteeringConfig&  get_steering_config()  { return steering_cfg; }
AvoidanceConfig& get_avoidance_config() { return avoidance_cfg; }
ArrivalConfig&   get_arrival_config()   { return arrival_cfg; }

void set_target_in_rect(const Rectangle& rect, const Vector3& pos) {
    for (int i = 0; i < agents.size(); ++i) {
        if (CheckCollisionPointRec({ agents.positions[i].x, agents.positions[i].y }, rect)) {
            agents.targets[i] = pos;
            agents.arrived[i] = 0;
        }
    }
}

void sim_tick(const float dt) {
    apply_steering (agents.positions, agents.targets, agents.vel, steering_cfg);
    apply_avoidance(agents.positions, agents.vel, avoidance_cfg);
    apply_arrival  (agents.positions, agents.targets, agents.vel, agents.arrived, arrival_cfg);
    apply_movement (agents.positions, agents.vel, dt);
    apply_separation(agents.positions, avoidance_cfg);
}

const std::vector<Vector3>& get_agent_positions() { return agents.positions; }
const std::vector<Vector3>& get_agent_targets()   { return agents.targets;   }
