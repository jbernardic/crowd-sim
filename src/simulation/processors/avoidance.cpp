#include "avoidance.h"
#include "../config.h"
#include "raymath.h"
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>

void apply_avoidance(
    AvoidanceContext&           ctx,
    const std::vector<Vector3>& positions,
    const std::vector<Vector3>& targets,
    std::vector<Vector3>&       vel,
    float                       dt)
{
    const auto& cfg = get_avoidance_config();
    if (!cfg.enabled) return;
    const float diameter = get_agent_config().radius * 2.0f;
    const float settle   = get_formation_config().settle_radius;

    const int n = (int)positions.size();
    if (n < 2) return;

    // Grow the persistent buffers for any newly spawned agents. New entries start
    // un-settled, with their current target recorded so they aren't immediately
    // flagged as having received a new order.
    if ((int)ctx.settled.size() < n) {
        int old = (int)ctx.settled.size();
        ctx.settled.resize(n, 0);
        ctx.last_target.resize(n);
        for (int i = old; i < n; ++i) ctx.last_target[i] = targets[i];
    }

    // Latched settle state. An agent becomes settled once it reaches its slot
    // (within the settle radius — the same test that draws it green) and STAYS
    // settled even if it is later nudged off the slot. Only a new order, detected
    // as its target changing, clears the latch so it travels (and re-settles) again.
    std::vector<char>& settled = ctx.settled;
    for (int i = 0; i < n; ++i) {
        const Vector3 t = targets[i], lt = ctx.last_target[i];
        if (t.x != lt.x || t.y != lt.y || t.z != lt.z) settled[i] = 0;
        ctx.last_target[i] = t;
        if (!settled[i] && Vector3Length(t - positions[i]) <= settle) settled[i] = 1;
    }

    // Spatial hash. Two agents can only collide if they are within `diameter` of
    // each other, so bucketing positions into cells of that size means a pair can
    // only touch when they share a cell or sit in adjacent cells. Each agent then
    // tests its 3x3 cell neighbourhood instead of every other agent, turning the
    // O(n^2) sweep into ~O(n) for the roughly uniform densities of a crowd. The
    // cell hash mirrors encode_tile, so it covers the unbounded world.
    const float cell = diameter > 0.001f ? diameter : 1.0f;
    const float inv  = 1.0f / cell;
    auto cell_x = [&](const Vector3& p) { return (int)floorf(p.x * inv); };
    auto cell_y = [&](const Vector3& p) { return (int)floorf(p.y * inv); };
    auto cell_key = [](int cx, int cy) -> uint64_t {
        return ((uint64_t)(uint32_t)cx << 32) | (uint64_t)(uint32_t)cy;
    };

    // Every agent goes in the grid (a settled agent must notice travelling agents
    // overlapping it), even though only settled agents are actually pushed.
    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(n);
    for (int i = 0; i < n; ++i)
        grid[cell_key(cell_x(positions[i]), cell_y(positions[i]))].push_back(i);

    for (int i = 0; i < n; ++i) {
        const int cx = cell_x(positions[i]), cy = cell_y(positions[i]);

        for (int ox = -1; ox <= 1; ++ox)
            for (int oy = -1; oy <= 1; ++oy) {
                auto it = grid.find(cell_key(cx + ox, cy + oy));
                if (it == grid.end()) continue;
                for (int j : it->second) {
                    // Act only on j > i so each unordered pair is handled once
                    // (and an agent never tests itself).
                    if (j <= i) continue;

                    Vector3 pi   = positions[i] + Vector3Scale(vel[i], dt);
                    Vector3 pj   = positions[j] + Vector3Scale(vel[j], dt);
                    Vector3 diff = pi - pj;
                    float dist = Vector3Length(diff);
                    if (dist < 0.001f) { diff = { 1.0f, 0.0f, 0.0f }; dist = 0.001f; }
                    if (dist >= diameter) continue;
                    const float   overlap = diameter - dist;
                    const Vector3 dir     = Vector3Normalize(diff);
                    const float   frac    = overlap / diameter;

                    if (!settled[i] && !settled[j]) {
                        // Both travelling: normal symmetric avoidance.
                        const Vector3 push = Vector3Scale(dir, cfg.strength * frac);
                        vel[i] += push;
                        vel[j] -= push;
                    } else if (settled[i] != settled[j]) {
                        // Exactly one settled: it steps aside (with its own gentler
                        // push) for the traveller, who passes through unshoved.
                        const Vector3 push = Vector3Scale(dir, cfg.settle_push * frac);
                        if (settled[i]) vel[i] += push; else vel[j] -= push;
                    }
                    // Both settled: no collision -- settled agents never push each other.
                }
            }
    }
}
