#include "arrival.h"
#include "../config.h"
#include "raymath.h"

// arrived == 0.0f  : moving
// arrived >  0.0f  : bumped, counting down
// arrived <  0.0f  : fully stopped

void apply_arrival(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    std::vector<float>&         arrived,
    float                       dt)
{
    const auto& cfg = get_arrival_config();
    if (!cfg.enabled) return;

    // direct arrival — stop immediately
    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i] != 0.0f) continue;
        if (Vector3Length(targets[i] - positions[i]) <= cfg.arrival_radius)
            arrived[i] = -1.0f;
    }

    // transitive bump — start countdown
    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i] != 0.0f) continue;
        for (int j = 0; j < (int)positions.size(); ++j) {
            if (arrived[j] == 0.0f) continue;
            if (Vector3Length(targets[i] - targets[j]) > 0.1f) continue;
            const float bump_radius = get_agent_config().agent_radius * 2.0f + get_steering_config().speed * dt;
            if (Vector3Length(positions[i] - positions[j]) <= bump_radius) {
                arrived[i] = cfg.bump_delay > 0.0f ? cfg.bump_delay : -1.0f;
                break;
            }
        }
    }

    // tick countdown; zero vel when fully stopped
    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i] > 0.0f) {
            arrived[i] -= dt;
            if (arrived[i] <= 0.0f)
                arrived[i] = -1.0f;
        }
        if (arrived[i] < 0.0f)
            vel[i] = { 0.0f, 0.0f, 0.0f };
    }
}
