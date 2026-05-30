#include "formation.h"
#include "../config.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
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

inline uint64_t cell_key(int cx, int cy) {
    return ((uint64_t)(uint32_t)cx << 32) | (uint64_t)(uint32_t)cy;
}

} // namespace

void apply_formation(
    const std::vector<Vector3>& positions,
    std::vector<Vector3>&       targets,
    std::vector<Vector3>&       nav_goal,
    const std::vector<int>&     members,
    const Vector3&              destination)
{
    const auto& cfg = get_formation_config();

    const int n = (int)members.size();
    if (n == 0) return;

    const float diameter = get_agent_config().radius * 2.0f;
    const float spacing   = cfg.slot_spacing * diameter;
    const float reach     = cfg.settle_radius;

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

    // Assign members to slots so the total squared travel distance is minimised
    // (that assignment is crossing-free, which is what makes the blob look tidy).
    //
    // Trick: minimising sum |pos_i - slot_s|^2 is identical to minimising it over
    // *centroid-relative* coordinates -- the constant translation between the two
    // clusters drops out of the sum. Centring both clouds makes them overlap no
    // matter how far the crowd is from the destination, so a local spatial search
    // finds the same answer the old all-pairs greedy + 2-opt did, but in ~O(n)
    // instead of ~O(n^2) per 2-opt pass iterated to convergence (the old stall).
    std::vector<Vector2> mrel(n), srel(n);
    for (int k = 0; k < n; ++k)
        mrel[k] = { positions[members[k]].x - centroid.x, positions[members[k]].y - centroid.y };
    for (int s = 0; s < n; ++s)
        srel[s] = { slots[s].x - anchor.x, slots[s].y - anchor.y };

    const float cell = spacing > 0.001f ? spacing : 1.0f;
    const float inv  = 1.0f / cell;
    auto cell_of = [&](float x, float y) {
        return cell_key((int)floorf(x * inv), (int)floorf(y * inv));
    };

    // Spatial hash of still-unclaimed members in relative space.
    std::unordered_map<uint64_t, std::vector<int>> mgrid;
    mgrid.reserve(n * 2);
    std::vector<uint64_t> mkey(n);
    for (int k = 0; k < n; ++k) {
        mkey[k] = cell_of(mrel[k].x, mrel[k].y);
        mgrid[mkey[k]].push_back(k);
    }
    auto remove_member = [&](int k) {
        auto& v = mgrid[mkey[k]];
        for (size_t t = 0; t < v.size(); ++t)
            if (v[t] == k) { v[t] = v.back(); v.pop_back(); break; }
    };

    std::vector<int> slot_member(n, -1);

    // Greedy: each slot (tip-out order) claims the nearest unclaimed member,
    // searching cells outward from the slot until no nearer member can exist.
    for (int s = 0; s < n; ++s) {
        const float sx = srel[s].x, sy = srel[s].y;
        const int qx = (int)floorf(sx * inv), qy = (int)floorf(sy * inv);
        int   best = -1;
        float bestd2 = 0.0f;

        auto scan = [&](int cx, int cy) {
            auto it = mgrid.find(cell_key(cx, cy));
            if (it == mgrid.end()) return;
            for (int k : it->second) {
                float dx = mrel[k].x - sx, dy = mrel[k].y - sy;
                float d2 = dx * dx + dy * dy;
                if (best == -1 || d2 < bestd2) { bestd2 = d2; best = k; }
            }
        };

        for (int R = 0; ; ++R) {
            // Any cell in ring R is at least (R-1)*cell away, so once the best
            // found is nearer than that the outer rings cannot beat it.
            if (best != -1) {
                float ringmin = (R - 1) * cell;
                if (ringmin > 0.0f && ringmin * ringmin > bestd2) break;
            }
            if (R == 0) {
                scan(qx, qy);
            } else {
                for (int dx = -R; dx <= R; ++dx) { scan(qx + dx, qy - R); scan(qx + dx, qy + R); }
                for (int dy = -R + 1; dy <= R - 1; ++dy) { scan(qx - R, qy + dy); scan(qx + R, qy + dy); }
            }
            // n members for n slots guarantees a free member always exists, so the
            // search is bounded; no explicit upper limit on R is needed.
        }
        slot_member[s] = best;
        remove_member(best);
    }

    // Local 2-opt: clear the few crossings greedy leaves by swapping members
    // between spatially-near slots when that shortens the travel. Restricting
    // swaps to neighbouring slot cells keeps each pass O(n); a handful converge.
    std::unordered_map<uint64_t, std::vector<int>> sgrid;
    sgrid.reserve(n * 2);
    for (int s = 0; s < n; ++s)
        sgrid[cell_of(srel[s].x, srel[s].y)].push_back(s);

    auto cost2 = [&](int k, int s) {
        float dx = mrel[k].x - srel[s].x, dy = mrel[k].y - srel[s].y;
        return dx * dx + dy * dy;
    };

    constexpr int MAX_PASSES = 6;
    for (int pass = 0; pass < MAX_PASSES; ++pass) {
        bool improved = false;
        for (int a = 0; a < n; ++a) {
            const int qx = (int)floorf(srel[a].x * inv), qy = (int)floorf(srel[a].y * inv);
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy) {
                    auto it = sgrid.find(cell_key(qx + dx, qy + dy));
                    if (it == sgrid.end()) continue;
                    for (int b : it->second) {
                        if (b <= a) continue;
                        const int ka = slot_member[a], kb = slot_member[b];
                        const float cur = cost2(ka, a) + cost2(kb, b);
                        const float swp = cost2(ka, b) + cost2(kb, a);
                        if (swp + 1e-4f < cur) {
                            slot_member[a] = kb;
                            slot_member[b] = ka;
                            improved = true;
                        }
                    }
                }
        }
        if (!improved) break;
    }

    // Assign goals: each agent's slot (local) + the shared anchor (global).
    for (int s = 0; s < n; ++s) {
        const Vector3& slot = slots[s];
        const int i = members[slot_member[s]];

        // Already standing on this slot — leave it be.
        if (Vector3Distance(positions[i], slot) <= reach) continue;

        targets[i]  = slot;
        nav_goal[i] = anchor;
    }
}
