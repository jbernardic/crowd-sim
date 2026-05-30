#include "pathfinding.h"
#include "walls.h"
#include "../config.h"
#include "raylib.h"
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>

// Flow-field pathfinding.
//
// destinations[i] is each agent's final goal; targets[i] is what steering
// chases. For every distinct destination cell we build a flow field once
// (Dijkstra from the goal across walkable cells) and cache it. Each tick an
// agent reads the flow vector at its cell and aims a short distance along it,
// so steering follows the field around walls. The cache is dropped whenever
// the walls, tile size or window dimensions change.

namespace {

struct Grid { int gw, gh; float ts; };

const int   DX[8]    = {  1, -1,  0,  0,  1,  1, -1, -1 };
const int   DY[8]    = {  0,  0,  1, -1,  1, -1,  1, -1 };
const float DCOST[8] = {  1, 1, 1, 1, 1.4142136f, 1.4142136f, 1.4142136f, 1.4142136f };

bool cell_blocked(int gx, int gy, float ts, const std::unordered_set<uint64_t>& walls) {
    const float wts = get_wall_config().tile_size;
    int wtx = (int)floorf(((gx + 0.5f) * ts) / wts);
    int wty = (int)floorf(((gy + 0.5f) * ts) / wts);
    return walls.count(encode_tile(wtx, wty)) != 0;
}

FlowField build_field(int goalGx, int goalGy, const Grid& g,
                      const std::unordered_set<uint64_t>& walls) {
    const int gw = g.gw, gh = g.gh, n = gw * gh;
    FlowField f;
    f.gw = gw; f.gh = gh;
    f.flow.assign(n, { 0.0f, 0.0f });

    auto inb = [&](int x, int y) { return x >= 0 && y >= 0 && x < gw && y < gh; };
    if (!inb(goalGx, goalGy)) return f;

    std::vector<uint8_t> blocked(n, 0);
    for (int y = 0; y < gh; ++y)
        for (int x = 0; x < gw; ++x)
            blocked[y * gw + x] = cell_blocked(x, y, g.ts, walls) ? 1 : 0;

    // Extra cost on cells next to a wall so paths keep their distance.
    const float wall_pen = get_pathfinding_config().wall_proximity_penalty;
    std::vector<float> pen(n, 0.0f);
    if (wall_pen > 0.0f) {
        for (int y = 0; y < gh; ++y)
            for (int x = 0; x < gw; ++x) {
                if (blocked[y * gw + x]) continue;
                for (int k = 0; k < 8; ++k) {
                    int nx = x + DX[k], ny = y + DY[k];
                    if (inb(nx, ny) && blocked[ny * gw + nx]) { pen[y * gw + x] = wall_pen; break; }
                }
            }
    }

    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> cost(n, INF);

    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    const int goalIdx = goalGy * gw + goalGx;
    cost[goalIdx] = 0.0f;
    pq.push({ 0.0f, goalIdx });

    auto diag_open = [&](int x, int y, int k) {
        // disallow squeezing diagonally between two walls
        return !(blocked[y * gw + (x + DX[k])] || blocked[(y + DY[k]) * gw + x]);
    };

    while (!pq.empty()) {
        auto [c, idx] = pq.top(); pq.pop();
        if (c > cost[idx]) continue;
        int x = idx % gw, y = idx / gw;
        for (int k = 0; k < 8; ++k) {
            int nx = x + DX[k], ny = y + DY[k];
            if (!inb(nx, ny)) continue;
            int nidx = ny * gw + nx;
            if (blocked[nidx]) continue;
            if (k >= 4 && !diag_open(x, y, k)) continue;
            float nc = c + DCOST[k] + pen[nidx];
            if (nc < cost[nidx]) { cost[nidx] = nc; pq.push({ nc, nidx }); }
        }
    }

    // Flow points toward the cheapest reachable neighbour.
    for (int y = 0; y < gh; ++y)
        for (int x = 0; x < gw; ++x) {
            int idx = y * gw + x;
            if (idx == goalIdx || blocked[idx] || cost[idx] == INF) continue;
            float best = cost[idx];
            int bx = 0, by = 0;
            for (int k = 0; k < 8; ++k) {
                int nx = x + DX[k], ny = y + DY[k];
                if (!inb(nx, ny)) continue;
                int nidx = ny * gw + nx;
                if (blocked[nidx] || cost[nidx] == INF) continue;
                if (k >= 4 && !diag_open(x, y, k)) continue;
                if (cost[nidx] < best) { best = cost[nidx]; bx = DX[k]; by = DY[k]; }
            }
            if (bx || by) {
                float len = sqrtf((float)(bx * bx + by * by));
                f.flow[idx] = { bx / len, by / len };
            }
        }
    return f;
}

// Bilinearly blend the four surrounding cells' flow vectors so the steering
// direction varies smoothly across the grid instead of snapping between the
// eight quantised directions at every cell boundary (the source of the
// movement flicker). Returns a unit vector, or {0,0} where there is no flow.
Vector2 sample_flow(const FlowField& f, float x, float y, float ts) {
    const float fx = x / ts - 0.5f;
    const float fy = y / ts - 0.5f;
    const int   x0 = (int)floorf(fx), y0 = (int)floorf(fy);
    const float tx = fx - x0,         ty = fy - y0;

    auto at = [&](int cx, int cy) -> Vector2 {
        if (cx < 0 || cy < 0 || cx >= f.gw || cy >= f.gh) return { 0.0f, 0.0f };
        return f.flow[cy * f.gw + cx];
    };
    const Vector2 v00 = at(x0, y0),     v10 = at(x0 + 1, y0);
    const Vector2 v01 = at(x0, y0 + 1), v11 = at(x0 + 1, y0 + 1);

    Vector2 a = { v00.x + (v10.x - v00.x) * tx, v00.y + (v10.y - v00.y) * tx };
    Vector2 b = { v01.x + (v11.x - v01.x) * tx, v01.y + (v11.y - v01.y) * tx };
    Vector2 v = { a.x + (b.x - a.x) * ty,        a.y + (b.y - a.y) * ty };

    const float len = sqrtf(v.x * v.x + v.y * v.y);
    if (len < 0.01f) return { 0.0f, 0.0f };
    return { v.x / len, v.y / len };
}

} // namespace

void apply_pathfinding(
    PathfindingContext&                 ctx,
    const std::vector<Vector3>&         positions,
    std::vector<Vector3>&               targets,
    const std::vector<Vector3>&         destinations,
    const std::vector<Vector3>&         nav_goal,
    const std::unordered_set<uint64_t>& wall_tiles)
{
    const auto& cfg = get_pathfinding_config();
    if (!cfg.enabled) return;

    int   W  = GetScreenWidth();  if (W <= 0) W = 1280;
    int   H  = GetScreenHeight(); if (H <= 0) H = 720;
    float ts = cfg.tile_size;     if (ts < 1.0f) ts = 1.0f;
    Grid  g{ (int)ceilf(W / ts), (int)ceilf(H / ts), ts };

    uint64_t wxor = 0;
    for (uint64_t k : wall_tiles) wxor ^= k;
    if (wxor != ctx.wall_xor || wall_tiles.size() != ctx.wall_count ||
        ts != ctx.built_ts || W != ctx.built_w || H != ctx.built_h) {
        ctx.cache.clear();
        ctx.wall_xor = wxor; ctx.wall_count = wall_tiles.size();
        ctx.built_ts = ts;   ctx.built_w = W; ctx.built_h = H;
    }

    auto to_cell = [&](float x, float y, int& cx, int& cy) {
        cx = (int)floorf(x / ts); cy = (int)floorf(y / ts);
        if (cx < 0) cx = 0; else if (cx >= g.gw) cx = g.gw - 1;
        if (cy < 0) cy = 0; else if (cy >= g.gh) cy = g.gh - 1;
    };

    // Aim past the arrival radius so the moving carrot never reads as "arrived".
    const float lookahead = get_arrival_config().arrival_radius + ts;

    for (int i = 0; i < (int)positions.size(); ++i) {
        const Vector3 pos    = positions[i];
        const Vector3 dest   = destinations[i];   // this agent's own slot
        const Vector3 anchor = nav_goal[i];       // shared formation anchor

        // Hand-off: once within the blob (its radius around the anchor, plus a
        // margin) steer straight to the slot through the open blob interior — no
        // field needed. By the triangle inequality this also covers "almost at
        // my slot", so the slot's own cell never needs a field of its own.
        const float bx = dest.x - anchor.x, by = dest.y - anchor.y;
        const float blob_r = sqrtf(bx * bx + by * by);
        const float ax = anchor.x - pos.x, ay = anchor.y - pos.y;
        if (sqrtf(ax * ax + ay * ay) <= blob_r + lookahead) { targets[i] = dest; continue; }

        // Otherwise follow the shared field toward the anchor cell. Every member
        // of a formation shares this cell, so the field is built and cached once.
        int dgx, dgy; to_cell(anchor.x, anchor.y, dgx, dgy);
        uint64_t key = encode_tile(dgx, dgy);
        auto it = ctx.cache.find(key);
        if (it == ctx.cache.end())
            it = ctx.cache.emplace(key, build_field(dgx, dgy, g, wall_tiles)).first;
        const FlowField& f = it->second;

        Vector2 d = sample_flow(f, pos.x, pos.y, ts);
        if (d.x == 0.0f && d.y == 0.0f)
            targets[i] = dest;                                  // goal cell or no path
        else
            targets[i] = { pos.x + d.x * lookahead, pos.y + d.y * lookahead, 0.0f };
    }
}
