#pragma once
#include "raylib.h"
#include <vector>

struct AgentData {
    std::vector<Vector3>  positions;
    std::vector<Vector3>  targets;       // local goal: the slot this agent settles on
    std::vector<Vector3>  nav_goal;      // global goal: shared formation pathfinding anchor
    std::vector<Vector3>  vel;

    int size() const { return static_cast<int>(positions.size()); }

    void add(const Vector3& pos) {
        positions.push_back(pos);
        targets.push_back(pos);
        nav_goal.push_back(pos);
        vel.push_back({ 0.0f, 0.0f, 0.0f });
    }
};
