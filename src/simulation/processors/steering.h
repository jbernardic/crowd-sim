#pragma once
#include "../config.h"
#include "raylib.h"
#include <vector>

struct SteeringConfig {
    bool  enabled = true;
    float speed   = 100.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Enabled", &enabled              },
            ConfigFloatField{ "Speed",   &speed, 1.0f, 500.0f },
        };
    }
};

void apply_steering(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    const SteeringConfig&       cfg);
