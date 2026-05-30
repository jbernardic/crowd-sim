#include "avoidance.h"
#include "../config.h"
#include "raymath.h"

void apply_avoidance(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    float                       dt)
{
    const auto& cfg = get_avoidance_config();
    if (!cfg.enabled) return;
    const float diameter = get_agent_config().radius * 2.0f;
    const float settle   = get_formation_config().settle_radius;

    const int n = (int)positions.size();

    // A unit that has reached its slot (within the settle radius — the same test
    // that draws it green) is fully non-colliding: it neither pushes nor is
    // pushed, against settled and travelling units alike. Units still on their
    // way pass straight through the settled blob to reach their own slots.
    std::vector<char> settled(n);
    if (get_formation_config().enabled)
    {
        for (int i = 0; i < n; ++i) {
            float d = Vector3Length(targets[i] - positions[i]);
            settled[i] = d <= settle ? 1 : 0;
        }   
    }

    for (int i = 0; i < n; ++i) {
        if (settled[i]) continue;
        for (int j = i + 1; j < n; ++j) {
            if (settled[j]) continue;
            Vector3 pi   = positions[i] + Vector3Scale(vel[i], dt);
            Vector3 pj   = positions[j] + Vector3Scale(vel[j], dt);
            Vector3 diff = pi - pj;
            float dist = Vector3Length(diff);
            if (dist < 0.001f) { diff = { 1.0f, 0.0f, 0.0f }; dist = 0.001f; }
            if (dist >= diameter) continue;
            const float overlap = diameter - dist;
            const Vector3 push = Vector3Scale(Vector3Normalize(diff), cfg.strength * (overlap / diameter));
            vel[i] += push;
            vel[j] -= push;
        }
    }
}
