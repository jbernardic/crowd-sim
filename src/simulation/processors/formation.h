#pragma once
#include "raylib.h"
#include <vector>

// Arranges a group of agents into a hex-packed "blob" centred on `destination`.
//
// Slot 0 sits exactly on `destination`; the remaining slots spiral outward in a
// hexagonal lattice so the group settles into a tight, round blob. Slots are
// matched to members greedily from the centre out, so the member nearest the
// destination wins slot 0, the next-nearest free member wins the next slot, and
// so on.
//
// Only members that have not already reached their assigned slot get a new
// destination (so agents already standing in place are left undisturbed). For
// those that are moved, the target is also redirected straight to the slot when
// it currently equals the destination (i.e. the agent is in its final approach
// and pathfinding would otherwise keep aiming at the old goal).
//
// Returns the slot positions (slot 0 first), so callers can visualise them.
std::vector<Vector3> apply_formation(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       targets,
    std::vector<Vector3>&       destinations,
    const std::vector<int>&     members,
    const Vector3&              destination);
