#pragma once
#include "raylib.h"
#include <vector>

void apply_arrival(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& destinations,
    std::vector<float>&         arrived);
