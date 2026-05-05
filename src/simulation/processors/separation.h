#pragma once
#include "raylib.h"
#include <vector>

void apply_separation(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       vel,
    float                       dt);
