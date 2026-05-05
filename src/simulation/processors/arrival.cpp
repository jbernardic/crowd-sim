#include "arrival.h"
#include "raymath.h"

void apply_arrival(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    std::vector<uint8_t>&       arrived,
    const ArrivalConfig&        cfg)
{
    if (!cfg.enabled) return;

    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i]) { vel[i] = { 0.0f, 0.0f, 0.0f }; continue; }
        if (Vector3Length(targets[i] - positions[i]) <= cfg.arrival_radius)
            arrived[i] = 1;
    }

    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i]) continue;
        for (int j = 0; j < (int)positions.size(); ++j) {
            if (!arrived[j]) continue;
            if (Vector3Length(targets[i] - targets[j]) > 0.1f) continue;
            if (Vector3Length(positions[i] - positions[j]) <= cfg.bump_radius) {
                arrived[i] = 1;
                break;
            }
        }
    }

    for (int i = 0; i < (int)positions.size(); ++i)
        if (arrived[i]) vel[i] = { 0.0f, 0.0f, 0.0f };
}
