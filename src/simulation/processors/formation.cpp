#include "formation.h"
#include "../config.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <vector>

// Build `count` blob slots on a pointy-top hexagonal lattice, packed *behind*
// `front` (the destination). `back` is the unit direction pointing away from
// the destination toward where the crowd is coming from; only lattice cells on
// that side are kept, so slot 0 lands exactly on the destination as the leading
// point and every other slot trails behind it — nobody is placed past the goal.
// Slots are sorted by distance from the front so the blob fills in from the tip
// outward. Spacing matches the packing the arrival blob settles into, so the
// formation looks like a naturally clumped crowd.
namespace {

std::vector<Vector3> build_blob_slots(const Vector3& front, const Vector3& back,
                                      float spacing, int count) {
    std::vector<Vector3> slots;
    if (count <= 0) return slots;

    const float h = 0.8660254f; // sqrt(3) / 2

    // Only the back half of the disk is usable, so grow the lattice radius until
    // enough cells survive the front-side cull.
    for (int R = 0; ; ++R) {
        slots.clear();
        for (int q = -R; q <= R; ++q)
            for (int r = -R; r <= R; ++r) {
                int hex_dist = (std::abs(q) + std::abs(q + r) + std::abs(r)) / 2;
                if (hex_dist > R) continue;
                Vector3 off = { spacing * (q + r * 0.5f), spacing * (h * r), 0.0f };
                // Drop cells in front of the destination (keep slot 0 itself).
                if (Vector3DotProduct(off, back) < -0.001f) continue;
                slots.push_back({ front.x + off.x, front.y + off.y, front.z });
            }
        if ((int)slots.size() >= count) break;
    }

    std::sort(slots.begin(), slots.end(), [&](const Vector3& a, const Vector3& b) {
        return Vector3LengthSqr(a - front) < Vector3LengthSqr(b - front);
    });

    slots.resize(count);
    return slots;
}

} // namespace

void assign_goals(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       targets,
    std::vector<Vector3>&       nav_goal,
    const std::vector<int>&     members,
    const Vector3&              destination,
    FormationOrder&             order)
{
    const auto& cfg = get_formation_config();

    const int n = (int)members.size();
    order.members.clear();
    order.slots.clear();
    if (n == 0) return;

    const float diameter = get_agent_config().radius * 2.0f;
    const float spacing   = cfg.slot_spacing * diameter;

    Vector3 centroid = { 0.0f, 0.0f, 0.0f };
    for (int k = 0; k < n; ++k) centroid = centroid + positions[members[k]];
    centroid = Vector3Scale(centroid, 1.0f / n);

    Vector3 back = centroid - destination;
    back = (Vector3Length(back) > 0.001f) ? Vector3Normalize(back)
                                          : Vector3{ 0.0f, 1.0f, 0.0f };

    const std::vector<Vector3> slots = build_blob_slots(destination, back, spacing, n);

    // One shared navigation anchor for the whole formation: the blob's centroid.
    // Every member paths toward this single point (so pathfinding builds one flow
    // field for the group), then peels off to its own slot for the final leg.
    Vector3 anchor = { 0.0f, 0.0f, 0.0f };
    for (const Vector3& sp : slots) anchor = anchor + sp;
    anchor = Vector3Scale(anchor, 1.0f / n);

    // FIX: Use Squared Distance to penalize overtakes and line intersections
    auto dist_sq = [&](int k, int s) {
        return Vector3LengthSqr(positions[members[k]] - slots[s]);
    };

    std::vector<int>  slot_member(n, -1);
    std::vector<char> taken(n, 0);

    // 1. FRONT-TO-BACK GREEDY INITIALIZATION (Using squared distance)
    for (int s = 0; s < n; ++s) {
        int best_k = -1;
        float best_d = 0.0f;
        for (int k = 0; k < n; ++k) {
            if (taken[k]) continue;
            float d = dist_sq(k, s);
            if (best_k == -1 || d < best_d) { 
                best_d = d; 
                best_k = k; 
            }
        }
        slot_member[s] = best_k;
        taken[best_k] = 1;
    }

    // 2. UNTANGLE CROSSINGS & OVERTAKES (2-OPT with squared distance)
    for (bool improved = true; improved; ) {
        improved = false;
        for (int a = 0; a < n; ++a) {
            for (int b = a + 1; b < n; ++b) {
                int ka = slot_member[a], kb = slot_member[b];
                float cur = dist_sq(ka, a) + dist_sq(kb, b);
                float swp = dist_sq(ka, b) + dist_sq(kb, a);
                
                // If swapping reduces the sum of squared distances, they were crossed
                if (swp + 1e-4f < cur) {
                    slot_member[a] = kb;
                    slot_member[b] = ka;
                    improved = true;
                }
            }
        }
    }

    // 3. RECORD THE ORDER: store each member's slot (aligned with members) and
    // the shared anchor, so apply_formation can peel members off as they arrive.
    order.members = members;
    order.slots.assign(n, Vector3{ 0.0f, 0.0f, 0.0f });
    order.anchor  = anchor;
    for (int s = 0; s < n; ++s)
        order.slots[slot_member[s]] = slots[s];

    // Point every member at the shared anchor (local target and global nav goal
    // both the blob centre) so they path toward the blob as one group. Each
    // member peels off to its own slot later, in apply_formation, once it has
    // actually reached the blob.
    for (int k = 0; k < n; ++k)
        targets[members[k]] = nav_goal[members[k]] = anchor;
}

void apply_formation(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       targets,
    const FormationOrder&       order)
{
    const float settle = get_formation_config().settle_radius;

    // Mirror navigation.cpp's hand-off: a member is "in range" once it gets
    // within the blob's radius (its slot's distance from the anchor) plus the
    // settle margin of the shared anchor. From then on it steers at its own slot
    // instead of the anchor.
    for (int k = 0; k < (int)order.members.size(); ++k) {
        const int     i      = order.members[k];
        const Vector3 slot   = order.slots[k];
        const float   blob_r = Vector3Distance(slot, order.anchor);

        if (Vector3Distance(positions[i], order.anchor) <= blob_r + settle)
            targets[i] = slot;
    }
}
