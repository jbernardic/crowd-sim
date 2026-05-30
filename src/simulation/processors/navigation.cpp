#include "navigation.h"
#include "walls.h"
#include "../config.h"
#include "raylib.h"
#include <algorithm>
#include <cmath>
#include <climits>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>

// Flow-field pathfinding + steering, combined.
//
// For every distinct nav-goal cell we build a flow field once (Dijkstra from the
// goal across walkable cells) and cache it. Each tick an agent reads the flow
// vector at its cell and aims a short distance along it, so steering follows the
// field around walls. The cache is dropped whenever the walls, tile size or
// window dimensions change.

namespace {

// A rectangular window of the world grid: gw x gh cells of size ts, with local
// cell (0,0) anchored at world cell (ox, oy).
struct Grid { int gw, gh; float ts; int ox, oy; };

const int   DX[8]    = {  1, -1,  0,  0,  1,  1, -1, -1 };
const int   DY[8]    = {  0,  0,  1, -1,  1, -1,  1, -1 };
const float DCOST[8] = {  1, 1, 1, 1, 1.4142136f, 1.4142136f, 1.4142136f, 1.4142136f };

bool cell_blocked(int gx, int gy, const Grid& g, const std::unordered_set<uint64_t>& walls) {
    const float wts = get_wall_config().tile_size;
    int wtx = (int)floorf((((gx + g.ox) + 0.5f) * g.ts) / wts);
    int wty = (int)floorf((((gy + g.oy) + 0.5f) * g.ts) / wts);
    return walls.count(encode_tile(wtx, wty)) != 0;
}

// goalWx/goalWy are world cell coordinates; the field is built over `g`'s window.
FlowField build_field(int goalWx, int goalWy, const Grid& g,
                      const std::unordered_set<uint64_t>& walls) {
    const int gw = g.gw, gh = g.gh, n = gw * gh;
    FlowField f;
    f.gw = gw; f.gh = gh; f.ox = g.ox; f.oy = g.oy;
    f.flow.assign(n, { 0.0f, 0.0f });

    const int goalGx = goalWx - g.ox, goalGy = goalWy - g.oy;   // world -> local
    auto inb = [&](int x, int y) { return x >= 0 && y >= 0 && x < gw && y < gh; };
    if (!inb(goalGx, goalGy)) return f;

    std::vector<uint8_t> blocked(n, 0);
    for (int y = 0; y < gh; ++y)
        for (int x = 0; x < gw; ++x)
            blocked[y * gw + x] = cell_blocked(x, y, g, walls) ? 1 : 0;

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
    const float fx = x / ts - 0.5f - f.ox;   // world -> local (fractional) cell
    const float fy = y / ts - 0.5f - f.oy;
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

void apply_navigation(
    PathfindingContext&                 ctx,
    const std::vector<Vector3>&         positions,
    const std::vector<Vector3>&         nav_goal,
    const std::vector<Vector3>&         targets,
    std::vector<Vector3>&               vel,
    const std::unordered_set<uint64_t>& wall_tiles)
{
    std::fill(vel.begin(), vel.end(), Vector3{ 0.0f, 0.0f, 0.0f });

    const int   n           = (int)positions.size();
    const float max_speed   = get_agent_config().speed;
    const bool  use_field   = get_pathfinding_config().enabled;
    const float slow_radius = get_formation_config().settle_radius;

    float ts = get_pathfinding_config().tile_size; if (ts < 1.0f) ts = 1.0f;

    // Aim the moving carrot a little past the slot radius so following the field
    // never reads as "settled".
    const float lookahead = slow_radius + ts;

    // Build the pathfinding window around wherever the agents actually are, so it
    // tracks the action across an unbounded world instead of being pinned to the
    // window. The region is the world-cell bounding box of every agent and its
    // nav goal, padded and snapped to chunks so it only changes occasionally.
    Grid g{ 0, 0, ts, 0, 0 };
    if (use_field && n > 0) {
        constexpr int MARGIN = 16;   // cells of slack around the agents
        constexpr int CHUNK  = 64;   // snap bounds to chunks so rebuilds are rare
        auto floordiv = [](int a, int b) { int q = a / b; if ((a % b) != 0 && ((a < 0) != (b < 0))) --q; return q; };
        auto cell     = [&](float v) { return (int)floorf(v / ts); };

        int minx = INT_MAX, miny = INT_MAX, maxx = INT_MIN, maxy = INT_MIN;
        for (int i = 0; i < n; ++i) {
            for (const Vector3& p : { positions[i], nav_goal[i] }) {
                int cx = cell(p.x), cy = cell(p.y);
                if (cx < minx) minx = cx;  if (cy < miny) miny = cy;
                if (cx > maxx) maxx = cx;  if (cy > maxy) maxy = cy;
            }
        }
        minx -= MARGIN; miny -= MARGIN; maxx += MARGIN; maxy += MARGIN;
        const int ox = floordiv(minx, CHUNK) * CHUNK;
        const int oy = floordiv(miny, CHUNK) * CHUNK;
        const int ex = (floordiv(maxx, CHUNK) + 1) * CHUNK;
        const int ey = (floordiv(maxy, CHUNK) + 1) * CHUNK;
        g = Grid{ ex - ox, ey - oy, ts, ox, oy };

        uint64_t wxor = 0;
        for (uint64_t k : wall_tiles) wxor ^= k;
        if (wxor != ctx.wall_xor || wall_tiles.size() != ctx.wall_count || ts != ctx.built_ts ||
            g.ox != ctx.region_ox || g.oy != ctx.region_oy ||
            g.gw != ctx.region_gw || g.gh != ctx.region_gh) {
            ctx.cache.clear();
            ctx.wall_xor = wxor; ctx.wall_count = wall_tiles.size(); ctx.built_ts = ts;
            ctx.region_ox = g.ox; ctx.region_oy = g.oy;
            ctx.region_gw = g.gw; ctx.region_gh = g.gh;
        }
    }

    for (int i = 0; i < n; ++i) {
        const Vector3 pos  = positions[i];
        const Vector3 slot = targets[i];   // local goal

        // The point we steer toward this tick. Default: straight to the slot.
        Vector3 steer = slot;

        if (use_field) {
            const Vector3 anchor = nav_goal[i];   // shared global goal

            // Hand-off: once within the blob (its radius around the anchor, plus a
            // margin) steer straight to the slot through the open blob interior.
            // By the triangle inequality this also covers "almost at my slot", so
            // a slot's own cell never needs a field of its own.
            const float bx = slot.x - anchor.x, by = slot.y - anchor.y;
            const float blob_r = sqrtf(bx * bx + by * by);
            const float ax = anchor.x - pos.x, ay = anchor.y - pos.y;
            if (sqrtf(ax * ax + ay * ay) > blob_r + lookahead) {
                // Follow the shared field toward the anchor's world cell. Every
                // member of a formation shares this cell, so it is built/cached
                // once. The cell is inside the region (nav goals are in its bbox).
                int dgx = (int)floorf(anchor.x / ts), dgy = (int)floorf(anchor.y / ts);
                uint64_t key = encode_tile(dgx, dgy);
                auto it = ctx.cache.find(key);
                if (it == ctx.cache.end())
                    it = ctx.cache.emplace(key, build_field(dgx, dgy, g, wall_tiles)).first;

                Vector2 d = sample_flow(it->second, pos.x, pos.y, ts);
                if (d.x != 0.0f || d.y != 0.0f)
                    steer = { pos.x + d.x * lookahead, pos.y + d.y * lookahead, 0.0f };
                // else no flow (goal cell / unreachable): keep steering at the slot.
            }
        }

        // Steer toward the carrot, easing off within the slot radius so a unit
        // settles gently (a passing unit can still nudge it; see avoidance).
        float dx = steer.x - pos.x, dy = steer.y - pos.y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist < 1.0f) continue;
        float speed = (dist < slow_radius) ? max_speed * (dist / slow_radius)
                                           : max_speed;
        vel[i] = { dx / dist * speed, dy / dist * speed, 0.0f };
    }
}
