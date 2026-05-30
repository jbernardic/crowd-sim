#include "arrival.h"
#include "../config.h"
#include "raymath.h"

// arrived == 0.0f : moving
// arrived == 1.0f : stopped (no goal-seeking; still a soft obstacle via avoidance)
//
// A moving unit stops when it either reaches its goal radius, or gets pressed
// against an already-stopped unit that shares its destination. The latter makes
// the settled blob grow outward from the target across frames.

void apply_arrival(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& destinations,
    std::vector<float>&         arrived)
{
    const auto& cfg = get_arrival_config();
    if (!cfg.enabled) return;

    const float diameter   = get_agent_config().agent_radius * 2.0f;
    const float bump_radius = cfg.stop_spacing * diameter;

    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i] != 0.0f) continue;

        // (a) reached the goal itself — seeds the blob
        if (Vector3Length(destinations[i] - positions[i]) <= cfg.arrival_radius) {
            arrived[i] = 1.0f;
            continue;
        }

        // (b) pressed against a settled neighbor heading to the same destination
        for (int j = 0; j < (int)positions.size(); ++j) {
            if (arrived[j] == 0.0f) continue;
            if (Vector3Length(destinations[i] - destinations[j]) > 0.1f) continue;
            if (Vector3Length(positions[i] - positions[j]) <= bump_radius) {
                arrived[i] = 1.0f;
                break;
            }
        }
    }
}
