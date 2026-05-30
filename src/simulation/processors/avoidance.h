#pragma once
#include "raylib.h"
#include <vector>

void apply_avoidance(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& destinations,
    std::vector<Vector3>&       vel,
    float                       dt);
