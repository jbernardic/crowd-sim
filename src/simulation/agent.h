#pragma once
#include "raylib.h"

struct Agent {
    Vector3 position;
    Vector3 target;
    bool has_target = false;
};
