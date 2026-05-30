#pragma once
#include <vector>
#include <variant>

// A single tunable value in a config module.
struct ConfigFloatField { const char* name; float* value; float min, max; };
struct ConfigBoolField  { const char* name; bool*  value; };
using ConfigField = std::variant<ConfigFloatField, ConfigBoolField>;

// ---------------------------------------------------------------------------
// Feature configs. Each is an independent module: a struct of tunable values,
// plus `enabled` where the feature can be switched off entirely. `fields()`
// lists only the tunables — the on/off toggle is handled at the module level
// (see ConfigModule) so the UI renders it consistently for every feature.
// ---------------------------------------------------------------------------

struct AgentConfig {
    float radius = 20.0f;
    float speed  = 100.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigFloatField{ "Radius", &radius, 1.0f,  60.0f  },
            ConfigFloatField{ "Speed",  &speed,  1.0f, 500.0f  },
        };
    }
};

struct PathfindingConfig {
    bool  enabled                = true;
    float tile_size              = 10.0f;
    float wall_proximity_penalty = 3.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigFloatField{ "Tile Size",              &tile_size,              4.0f, 64.0f },
            ConfigFloatField{ "Wall Proximity Penalty", &wall_proximity_penalty, 0.0f, 20.0f },
        };
    }
};

struct AvoidanceConfig {
    bool  enabled  = true;
    float strength = 300.0f;

    std::vector<ConfigField> fields() {
        return { ConfigFloatField{ "Strength", &strength, 0.0f, 1000.0f } };
    }
};

// Formation is core to group movement (every order spreads units into distinct
// slots), so it has no on/off toggle — only tuning. The arrangement *shape* is
// the intended modular axis (blob today; line/column/box later).
struct FormationConfig {
    float settle_radius = 20.0f;   // how close to a slot counts as "arrived" (also the steering ease-in radius)
    float slot_spacing  = 1.1f;    // formation slot spacing, in agent diameters

    std::vector<ConfigField> fields() {
        return {
            ConfigFloatField{ "Settle Radius", &settle_radius, 1.0f, 50.0f },
            ConfigFloatField{ "Slot Spacing",  &slot_spacing,  0.5f,  2.0f },
        };
    }
};

struct WallConfig {
    bool  has_collision = true;   // gates apply_wall_collision only; walls still block pathfinding
    float tile_size     = 20.0f;

    std::vector<ConfigField> fields() {
        return {
            ConfigBoolField{  "Has Collision", &has_collision         },
            ConfigFloatField{ "Tile Size",     &tile_size,    4.0f, 64.0f },
        };
    }
};

// ---------------------------------------------------------------------------
// Module registry. A ConfigModule wraps a feature for display/control: a name,
// an optional on/off toggle (`enabled` is null for always-on, tuning-only
// modules), and its tunable fields. `config_modules()` is the single list the
// UI walks — adding a feature is one line here, no UI changes needed.
// ---------------------------------------------------------------------------

struct ConfigModule {
    const char*              name;
    bool*                    enabled;   // null => no on/off toggle (always on)
    std::vector<ConfigField> fields;
};

std::vector<ConfigModule> config_modules();

// Direct accessors used by the simulation.
AgentConfig&       get_agent_config();
PathfindingConfig& get_pathfinding_config();
AvoidanceConfig&   get_avoidance_config();
FormationConfig&   get_formation_config();
WallConfig&        get_wall_config();
