#pragma once
#include "raylib.h"
#include <vector>

void apply_steering(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel);
