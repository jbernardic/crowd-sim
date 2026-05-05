#include "movement.h"
#include "raymath.h"

void apply_movement(
    std::vector<Vector3>&       positions,
    const std::vector<Vector3>& vel,
    float                       dt)
{
    for (int i = 0; i < (int)positions.size(); ++i)
        positions[i] += Vector3Scale(vel[i], dt);
}
