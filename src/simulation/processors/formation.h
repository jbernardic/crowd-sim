#pragma once
#include "raylib.h"
#include <vector>

// A group of agents arranged into a hex-packed "blob" centred on a destination.
//
// Slot 0 sits exactly on the destination; the remaining slots spiral outward in
// a hexagonal lattice so the group settles into a tight, round blob. Each member
// is matched to one slot, and every member shares a single navigation anchor
// (the blob centroid) so pathfinding builds one flow field for the whole group.
//
// The order is filled in two phases:
//   * assign_goals  — run once when the order is issued. Builds the blob, binds
//                     each member to a slot, and points everyone at the shared
//                     anchor (targets = nav_goal = anchor) so they path toward
//                     the blob as one group.
//   * apply_formation — run every tick. Once a member reaches the blob it is
//                     peeled off to its own slot (targets = slot), mirroring the
//                     in-range hand-off in navigation.cpp.
struct FormationOrder {
    std::vector<int>     members;   // agent indices that belong to this order
    std::vector<Vector3> slots;     // slots[k] is members[k]'s assigned slot
    Vector3              anchor{};  // shared flow-field goal (the blob centre)
};

// One-shot. Build the blob, assign each member a slot, point everyone at the
// shared anchor, and record the order so apply_formation can fill it in.
void assign_goals(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       targets,
    std::vector<Vector3>&       nav_goal,
    const std::vector<int>&     members,
    const Vector3&              destination,
    FormationOrder&             order);

// Every tick. For each member that has reached the blob, redirect its target to
// its own slot. Members still far away keep aiming at the anchor.
void apply_formation(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       targets,
    const FormationOrder&       order);
