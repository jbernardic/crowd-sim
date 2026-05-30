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

std::vector<Vector3> apply_formation(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       targets,
    std::vector<Vector3>&       destinations,
    std::vector<float>&         arrived,
    const std::vector<int>&     members,
    const Vector3&              destination)
{
    const int n = (int)members.size();
    if (n == 0) return {};

    const float diameter = get_agent_config().agent_radius * 2.0f;
    const float spacing   = get_arrival_config().stop_spacing * diameter;
    const float reach     = get_arrival_config().arrival_radius;

    // "Back" points from the destination toward the crowd's centre, so the blob
    // grows backward along the approach instead of overshooting the goal.
    Vector3 centroid = { 0.0f, 0.0f, 0.0f };
    for (int k = 0; k < n; ++k) centroid = centroid + positions[members[k]];
    centroid = Vector3Scale(centroid, 1.0f / n);

    Vector3 back = centroid - destination;
    back = (Vector3Length(back) > 0.001f) ? Vector3Normalize(back)
                                          : Vector3{ 0.0f, 1.0f, 0.0f };

    const std::vector<Vector3> slots = build_blob_slots(destination, back, spacing, n);

    // Greedy centre-out assignment: for each slot (nearest the destination
    // first) grab the closest member not yet placed. Slot 0 is the destination,
    // so the agent nearest the destination is the one that ends up standing on
    // it.
    std::vector<char> taken(n, 0);
    for (const Vector3& slot : slots) {
        int   best = -1;
        float best_d = 0.0f;
        for (int k = 0; k < n; ++k) {
            if (taken[k]) continue;
            float d = Vector3LengthSqr(positions[members[k]] - slot);
            if (best == -1 || d < best_d) { best = k; best_d = d; }
        }
        taken[best] = 1;

        const int i = members[best];

        // Already standing on this slot — leave it be.
        if (Vector3Length(positions[i] - slot) <= reach) continue;

        const bool target_was_dest =
            Vector3Length(targets[i] - destinations[i]) <= 0.1f;

        destinations[i] = slot;
        arrived[i]      = 0.0f;
        if (target_was_dest) targets[i] = slot;
    }

    return slots;
}
