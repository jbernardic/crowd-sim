#pragma once
#include "raylib.h"
#include <vector>

void apply_arrival(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    std::vector<float>&         arrived,
    float                       dt);