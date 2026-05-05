#include "separation.h"
#include "../config.h"
#include "raymath.h"

void apply_separation(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       vel,
    float                       dt)
{
    const auto& cfg = get_avoidance_config();
    if (!cfg.enabled) return;
    const float diameter = get_agent_config().agent_radius * 2.0f;
    for (int i = 0; i < (int)positions.size(); ++i) {
        for (int j = i + 1; j < (int)positions.size(); ++j) {
            Vector3 pi   = positions[i] + Vector3Scale(vel[i], dt);
            Vector3 pj   = positions[j] + Vector3Scale(vel[j], dt);
            Vector3 diff = pi - pj;
            float dist = Vector3Length(diff);
            if (dist < 0.001f) { diff = { 1.0f, 0.0f, 0.0f }; dist = 0.001f; }
            if (dist >= diameter) continue;
            Vector3 impulse = Vector3Scale(Vector3Normalize(diff), (diameter - dist) * 0.5f / dt);
            vel[i] += impulse;
            vel[j] -= impulse;
        }
    }
}
