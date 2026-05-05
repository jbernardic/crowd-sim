#include "steering.h"
#include "raymath.h"

void apply_steering(const std::vector<Agent>& agents, std::vector<Vector3>& vel, const SteeringConfig& cfg) {
    if (!cfg.enabled) return;
    for (int i = 0; i < (int)agents.size(); ++i) {
        if (!agents[i].has_target) continue;
        Vector3 delta = agents[i].target - agents[i].position;
        if (Vector3Length(delta) >= 1.0f)
            vel[i] += Vector3Scale(Vector3Normalize(delta), cfg.speed);
    }
}
