#pragma once
#include "raylib.h"
#include <vector>

struct AgentData {
    std::vector<Vector3>  positions;
    std::vector<Vector3>  targets;
    std::vector<Vector3>  destinations;
    std::vector<Vector3>  vel;

    int size() const { return static_cast<int>(positions.size()); }

    void add(const Vector3& pos) {
        positions.push_back(pos);
        destinations.push_back(pos);
        targets.push_back(pos);
        vel.push_back({ 0.0f, 0.0f, 0.0f });
    }
};
