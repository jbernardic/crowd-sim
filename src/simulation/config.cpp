#include "config.h"

static AgentConfig       agent_cfg;
static PathfindingConfig pathfinding_cfg;
static AvoidanceConfig   avoidance_cfg;
static FormationConfig   formation_cfg;
static WallConfig        wall_cfg;

AgentConfig&       get_agent_config()       { return agent_cfg;       }
PathfindingConfig& get_pathfinding_config() { return pathfinding_cfg; }
AvoidanceConfig&   get_avoidance_config()   { return avoidance_cfg;   }
FormationConfig&   get_formation_config()   { return formation_cfg;   }
WallConfig&        get_wall_config()        { return wall_cfg;        }

std::vector<ConfigModule> config_modules() {
    return {
        { "Agent",       nullptr,                  agent_cfg.fields()       },
        { "Formation",   nullptr,                  formation_cfg.fields()   },
        { "Pathfinding", &pathfinding_cfg.enabled, pathfinding_cfg.fields() },
        { "Avoidance",   &avoidance_cfg.enabled,   avoidance_cfg.fields()   },
        { "Walls",       nullptr,                  wall_cfg.fields()        },
    };
}
