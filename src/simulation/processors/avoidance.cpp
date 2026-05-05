#include "avoidance.h"
#include "raymath.h"

void apply_avoidance(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       vel,
    const AvoidanceConfig&      cfg)
{
    if (!cfg.enabled) return;
    const float diameter = cfg.agent_radius * 2.0f;
    for (int i = 0; i < (int)positions.size(); ++i) {
        for (int j = i + 1; j < (int)positions.size(); ++j) {
            Vector3 diff = positions[i] - positions[j];
            float dist = Vector3Length(diff);
            if (dist < 0.001f) { diff = { 1.0f, 0.0f, 0.0f }; dist = 0.001f; }
            if (dist >= diameter) continue;
            const float overlap = diameter - dist;
            const Vector3 push = Vector3Scale(Vector3Normalize(diff), cfg.avoidance_strength * (overlap / diameter));
            vel[i] += push;
            vel[j] -= push;
        }
    }
}

void apply_separation(
    std::vector<Vector3>&  positions,
    const AvoidanceConfig& cfg)
{
    if (!cfg.enabled) return;
    const float diameter = cfg.agent_radius * 2.0f;
    for (int i = 0; i < (int)positions.size(); ++i) {
        for (int j = i + 1; j < (int)positions.size(); ++j) {
            Vector3 diff = positions[i] - positions[j];
            float dist = Vector3Length(diff);
            if (dist < 0.001f) { diff = { 1.0f, 0.0f, 0.0f }; dist = 0.001f; }
            if (dist >= diameter) continue;
            Vector3 correction = Vector3Scale(Vector3Normalize(diff), (diameter - dist) * 0.5f);
            positions[i] += correction;
            positions[j] -= correction;
        }
    }
}
