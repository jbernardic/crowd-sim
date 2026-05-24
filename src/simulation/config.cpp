#include "config.h"

static PathfindingConfig pathfinding_cfg;
static AgentConfig       agent_cfg;
static SteeringConfig  steering_cfg;
static AvoidanceConfig avoidance_cfg;
static SeparationConfig seperation_cfg;
static ArrivalConfig   arrival_cfg;
static WallConfig      wall_cfg;

PathfindingConfig& get_pathfinding_config() { return pathfinding_cfg; }
AgentConfig&       get_agent_config()       { return agent_cfg;       }
SteeringConfig&  get_steering_config()  { return steering_cfg;  }
AvoidanceConfig& get_avoidance_config() { return avoidance_cfg; }
ArrivalConfig&   get_arrival_config()   { return arrival_cfg;   }
WallConfig&      get_wall_config()      { return wall_cfg;      }
SeparationConfig& get_seperation_config() { return seperation_cfg;  }
