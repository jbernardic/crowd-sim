#pragma once
#include "raylib.h"
#include "processors/steering.h"
#include "processors/avoidance.h"
#include <vector>

void sim_tick(float dt);
const std::vector<Vector3> get_agent_positions();
const std::vector<Vector3> get_agent_targets();
void set_target_in_rect(const Rectangle& rect, const Vector3& pos);

SteeringConfig&  get_steering_config();
AvoidanceConfig& get_avoidance_config();
