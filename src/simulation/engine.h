#pragma once
#include "raylib.h"
#include "config.h"
#include "processors/walls.h"
#include <unordered_set>
#include <cstdint>
#include <vector>

void sim_tick(float dt);
const std::vector<Vector3>& get_agent_positions();
const std::vector<Vector3>& get_agent_targets();
void set_target_in_rect(const Rectangle& rect, const Vector3& pos);
const std::vector<Vector3>& get_formation_slots();

void add_wall_tile(int x, int y);
const std::unordered_set<uint64_t>& get_wall_tiles();
