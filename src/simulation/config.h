#pragma once
#include <vector>
#include <variant>

struct ConfigFloatField { const char* name; float* value; float min, max; };
struct ConfigBoolField  { const char* name; bool*  value; };
using ConfigField = std::variant<ConfigFloatField, ConfigBoolField>;

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

struct AgentConfig {
    float agent_radius = 20.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigFloatField{ "Agent Radius", &agent_radius, 1.0f, 60.0f },
        };
    }
};

struct AvoidanceConfig {
    bool  enabled            = true;
    float avoidance_strength = 300.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Enabled",            &enabled                            },
            ConfigFloatField{ "Avoidance Strength",  &avoidance_strength, 0.0f, 1000.0f },
        };
    }
};


struct ArrivalConfig {
    bool  enabled        = true;
    float arrival_radius = 20.0f;
    float bump_delay     = 0.3f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Enabled",        &enabled                        },
            ConfigFloatField{ "Arrival Radius",  &arrival_radius, 1.0f,  50.0f },
            ConfigFloatField{ "Bump Delay",      &bump_delay,     0.0f,   3.0f },
        };
    }
};

struct WallConfig {
    bool  enabled          = true;
    float tile_size        = 20.0f;
    float agent_radius     = 20.0f;
    float avoidance_radius = 60.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Enabled",          &enabled                          },
            ConfigFloatField{ "Tile Size",        &tile_size,        4.0f,   64.0f },
            ConfigFloatField{ "Agent Radius",     &agent_radius,     1.0f,   60.0f },
            ConfigFloatField{ "Avoidance Radius", &avoidance_radius, 1.0f,  200.0f },
        };
    }
};

struct PathfindingConfig {
    bool  enabled               = true;
    float tile_size             = 10.0f;
    float wall_proximity_penalty = 3.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Enabled",                &enabled                              },
            ConfigFloatField{ "Tile Size",              &tile_size,              4.0f, 64.0f  },
            ConfigFloatField{ "Wall Proximity Penalty", &wall_proximity_penalty, 0.0f, 20.0f  },
        };
    }
};

// --- Getters ---

PathfindingConfig& get_pathfinding_config();
AgentConfig&       get_agent_config();
SteeringConfig&  get_steering_config();
AvoidanceConfig& get_avoidance_config();
ArrivalConfig&   get_arrival_config();
WallConfig&      get_wall_config();
