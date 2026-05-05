#include "movement.h"
#include "raymath.h"

void apply_movement(std::vector<Agent>& agents, const std::vector<Vector3>& vel, float dt) {
    for (int i = 0; i < (int)agents.size(); ++i)
        agents[i].position += Vector3Scale(vel[i], dt);
}
