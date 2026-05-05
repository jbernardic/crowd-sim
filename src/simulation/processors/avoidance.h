#pragma once
#include "../config.h"
#include "raylib.h"
#include <vector>

struct AvoidanceConfig {
    bool  enabled            = true;
    float agent_radius       = 20.0f;
    float avoidance_strength = 300.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Enabled",            &enabled                            },
            ConfigFloatField{ "Agent Radius",        &agent_radius,       1.0f,   60.0f },
            ConfigFloatField{ "Avoidance Strength",  &avoidance_strength, 0.0f, 1000.0f },
        };
    }
};

void apply_avoidance(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       vel,
    const AvoidanceConfig&      cfg);

void apply_separation(
    std::vector<Vector3>&  positions,
    const AvoidanceConfig& cfg);
