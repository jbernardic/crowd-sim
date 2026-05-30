#include "avoidance.h"
#include "../config.h"
#include "raymath.h"
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>

void apply_avoidance(
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

    // A unit that has reached its slot (within the settle radius — the same test
    // that draws it green) is fully non-colliding: it neither pushes nor is
    // pushed, against settled and travelling units alike. Units still on their
    // way pass straight through the settled blob to reach their own slots.
    std::vector<char> settled(n);
    for (int i = 0; i < n; ++i) {
        float d = Vector3Length(targets[i] - positions[i]);
        settled[i] = d <= settle ? 1 : 0;
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

    // Only travelling agents go in the grid; settled ones are skipped entirely.
    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (settled[i]) continue;
        grid[cell_key(cell_x(positions[i]), cell_y(positions[i]))].push_back(i);
    }

    for (int i = 0; i < n; ++i) {
        if (settled[i]) continue;
        const int cx = cell_x(positions[i]), cy = cell_y(positions[i]);

        for (int ox = -1; ox <= 1; ++ox)
            for (int oy = -1; oy <= 1; ++oy) {
                auto it = grid.find(cell_key(cx + ox, cy + oy));
                if (it == grid.end()) continue;
                for (int j : it->second) {
                    // Act only on j > i so each unordered pair is handled once
                    // (and an agent never tests itself), matching the symmetric
                    // push of the original all-pairs loop.
                    if (j <= i) continue;

                    Vector3 pi   = positions[i] + Vector3Scale(vel[i], dt);
                    Vector3 pj   = positions[j] + Vector3Scale(vel[j], dt);
                    Vector3 diff = pi - pj;
                    float dist = Vector3Length(diff);
                    if (dist < 0.001f) { diff = { 1.0f, 0.0f, 0.0f }; dist = 0.001f; }
                    if (dist >= diameter) continue;
                    const float overlap = diameter - dist;
                    const Vector3 push = Vector3Scale(Vector3Normalize(diff), cfg.strength * (overlap / diameter));
                    vel[i] += push;
                    vel[j] -= push;
                }
            }
    }
}
