#include "steering.h"
#include "../config.h"
#include "raymath.h"

void apply_steering(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    const std::vector<float>&   arrived)
{
    const auto& cfg = get_steering_config();
    std::fill(vel.begin(), vel.end(), Vector3{ 0.0f, 0.0f, 0.0f });
    if (!cfg.enabled) return;
    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i] != 0.0f) continue;
        Vector3 delta = targets[i] - positions[i];
        if (Vector3Length(delta) >= 1.0f)
            vel[i] = Vector3Scale(Vector3Normalize(delta), cfg.speed);
    }
}
