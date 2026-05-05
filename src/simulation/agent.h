#pragma once
#include "raylib.h"
#include <vector>

struct AgentData {
    std::vector<Vector3>  positions;
    std::vector<Vector3>  targets;
    std::vector<Vector3>  vel;
    std::vector<uint8_t>  arrived;

    int size() const { return (int)positions.size(); }

    void add(Vector3 pos) {
        positions.push_back(pos);
        targets.push_back(pos);
        vel.push_back({ 0.0f, 0.0f, 0.0f });
        arrived.push_back(0);
    }
};
