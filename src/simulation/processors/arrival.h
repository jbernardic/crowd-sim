#pragma once
#include "../config.h"
#include "raylib.h"
#include <vector>

struct ArrivalConfig {
    bool  enabled        = true;
    float arrival_radius = 10.0f;
    float bump_radius    = 40.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Enabled",        &enabled                         },
            ConfigFloatField{ "Arrival Radius",  &arrival_radius,  1.0f,  50.0f },
            ConfigFloatField{ "Bump Radius",     &bump_radius,     1.0f, 100.0f },
        };
    }
};

void apply_arrival(
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    std::vector<uint8_t>&       arrived,
    const ArrivalConfig&        cfg);
