#include "steering.h"
#include "../config.h"
#include "raymath.h"

void apply_steering(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel)
{
    const auto& cfg = get_steering_config();
    std::fill(vel.begin(), vel.end(), Vector3{ 0.0f, 0.0f, 0.0f });
    if (!cfg.enabled) return;

    // Ease off within the settle radius so a unit sitting in its slot only holds
    // it with a gentle force. A passing unit's avoidance push (much stronger
    // than this soft pull) shoves it aside easily, and it drifts back to the
    // slot once the lane clears.
    const float slow_radius = get_arrival_config().arrival_radius;

    for (int i = 0; i < (int)positions.size(); ++i) {
        Vector3 delta = targets[i] - positions[i];
        float dist = Vector3Length(delta);
        if (dist < 1.0f) continue;
        float speed = (dist < slow_radius) ? cfg.speed * (dist / slow_radius)
                                           : cfg.speed;
        vel[i] = Vector3Scale(Vector3Normalize(delta), speed);
    }
}
