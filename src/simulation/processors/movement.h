#pragma once
#include "../agent.h"
#include <vector>

void apply_movement(std::vector<Agent>& agents, const std::vector<Vector3>& vel, float dt);
