#include "pathfinding.h"
#include "../config.h"
#include "raymath.h"
#include <cmath>

static const int   DX[]   = { 1,-1, 0, 0, 1, 1,-1,-1 };
static const int   DY[]   = { 0, 0, 1,-1, 1,-1, 1,-1 };
static const float COST[] = { 1, 1, 1, 1, 1.414f, 1.414f, 1.414f, 1.414f };

// Check if a pathfinding tile (in pathfinding tile space) overlaps any wall tile
static bool is_wall(
    int tx, int ty,
    const std::unordered_set<uint64_t>& walls,
    float pts) // pathfinding tile size
{
    const float wts = get_wall_config().tile_size;
    int wx0 = (int)floorf(tx       * pts / wts);
    int wx1 = (int)floorf((tx + 1) * pts / wts);
    int wy0 = (int)floorf(ty       * pts / wts);
    int wy1 = (int)floorf((ty + 1) * pts / wts);
    for (int wx = wx0; wx <= wx1; ++wx)
        for (int wy = wy0; wy <= wy1; ++wy)
            if (walls.count(encode_tile(wx, wy))) return true;
    return false;
}

// DDA raycast using wall tile size — true if no wall blocks the line from a to b
static bool has_los(
    Vector2                              a,
    Vector2                              b,
    const std::unordered_set<uint64_t>& walls)
{
    const float ts = get_wall_config().tile_size;
    float dx = b.x - a.x, dy = b.y - a.y;
    if (fabsf(dx) < 0.001f && fabsf(dy) < 0.001f) return true;

    int tx = (int)floorf(a.x / ts), ty = (int)floorf(a.y / ts);
    int gx = (int)floorf(b.x / ts), gy = (int)floorf(b.y / ts);

    int   stepX   = dx > 0 ? 1 : -1, stepY = dy > 0 ? 1 : -1;
    float tDeltaX = fabsf(dx) > 0.001f ? fabsf(ts / dx) : 1e9f;
    float tDeltaY = fabsf(dy) > 0.001f ? fabsf(ts / dy) : 1e9f;
    float tMaxX   = fabsf(dx) > 0.001f ? (dx > 0 ? (tx+1)*ts - a.x : a.x - tx*ts) / fabsf(dx) : 1e9f;
    float tMaxY   = fabsf(dy) > 0.001f ? (dy > 0 ? (ty+1)*ts - a.y : a.y - ty*ts) / fabsf(dy) : 1e9f;

    while (true) {
        if (walls.count(encode_tile(tx, ty))) return false;
        if (tx == gx && ty == gy)             return true;
        if (tMaxX < tMaxY) { tMaxX += tDeltaX; tx += stepX; }
        else               { tMaxY += tDeltaY; ty += stepY; }
    }
}

static std::unordered_map<uint64_t, FlowField> ff_cache;
static std::unordered_map<int, uint64_t>        agent_goal;
static std::vector<Vector3>                      waypoints;

static FlowField make_flow_field(int gx, int gy) {
    FlowField ff;
    uint64_t gk = encode_tile(gx, gy);
    ff.dist[gk] = 0.0f;
    ff.frontier.push({ 0.0f, gk });
    return ff;
}

static void expand_to(
    FlowField&                           ff,
    uint64_t                             target_key,
    const std::unordered_set<uint64_t>& walls,
    float                                pts)
{
    if (ff.exhausted || ff.dir.count(target_key)) return;

    while (!ff.frontier.empty()) {
        auto [d, ck] = ff.frontier.top(); ff.frontier.pop();
        if (ff.dist.count(ck) && ff.dist[ck] < d) continue;
        if (ff.dir.count(ck))                      continue;

        int cx, cy; decode_tile(ck, cx, cy);
        float   best     = d;
        Vector2 best_dir = { 0.0f, 0.0f };
        for (int i = 0; i < 8; ++i) {
            uint64_t nk = encode_tile(cx + DX[i], cy + DY[i]);
            auto it = ff.dist.find(nk);
            if (it == ff.dist.end() || it->second >= best) continue;
            best     = it->second;
            float len = sqrtf((float)(DX[i]*DX[i] + DY[i]*DY[i]));
            best_dir = { DX[i] / len, DY[i] / len };
        }
        ff.dir[ck] = best_dir;

        for (int i = 0; i < 8; ++i) {
            int nx = cx + DX[i], ny = cy + DY[i];
            if (is_wall(nx, ny, walls, pts)) continue;
            if (i >= 4) {
                if (is_wall(cx + DX[i], cy, walls, pts) ||
                    is_wall(cx, cy + DY[i], walls, pts)) continue;
            }
            uint64_t nk = encode_tile(nx, ny);
            float ng = d + COST[i];
            for (int w = 0; w < 8; ++w)
                if (is_wall(nx + DX[w], ny + DY[w], walls, pts))
                    { ng += get_pathfinding_config().wall_proximity_penalty; break; }
            if (!ff.dist.count(nk) || ng < ff.dist[nk]) {
                ff.dist[nk] = ng;
                ff.frontier.push({ ng, nk });
            }
        }

        if (ck == target_key) return;
    }

    ff.exhausted = true;
}

void pathfinding_invalidate() {
    ff_cache.clear();
    agent_goal.clear();
}

void apply_pathfinding(
    const std::vector<Vector3>&         positions,
    const std::vector<Vector3>&         targets,
    const std::vector<float>&           arrived,
    const std::unordered_set<uint64_t>& wall_tiles)
{
    const auto& cfg = get_pathfinding_config();
    waypoints.resize(positions.size());

    if (!cfg.enabled) {
        for (int i = 0; i < (int)positions.size(); ++i)
            waypoints[i] = targets[i];
        return;
    }

    const float ts = cfg.tile_size;

    std::unordered_set<uint64_t> active_goals;

    for (int i = 0; i < (int)positions.size(); ++i) {
        if (arrived[i] != 0.0f) {
            waypoints[i] = targets[i];
            continue;
        }

        float dx = targets[i].x - positions[i].x;
        float dy = targets[i].y - positions[i].y;

        if (sqrtf(dx*dx + dy*dy) <= ts ||
            has_los({ positions[i].x, positions[i].y },
                    { targets[i].x,   targets[i].y   },
                    wall_tiles))
        {
            waypoints[i] = targets[i];
            continue;
        }

        int      gx = (int)floorf(targets[i].x / ts);
        int      gy = (int)floorf(targets[i].y / ts);
        uint64_t gk = encode_tile(gx, gy);

        active_goals.insert(gk);

        if (!ff_cache.count(gk))
            ff_cache.emplace(gk, make_flow_field(gx, gy));

        agent_goal[i] = gk;
        FlowField& ff = ff_cache[gk];

        int      cx = (int)floorf(positions[i].x / ts);
        int      cy = (int)floorf(positions[i].y / ts);
        uint64_t ck = encode_tile(cx, cy);

        expand_to(ff, ck, wall_tiles, ts);

        if (ff.dir.count(ck)) {
            const Vector2& dir = ff.dir.at(ck);
            waypoints[i] = { positions[i].x + dir.x * ts * 2.0f,
                             positions[i].y + dir.y * ts * 2.0f,
                             0.0f };
        } else {
            waypoints[i] = targets[i];
        }
    }

    for (auto it = ff_cache.begin(); it != ff_cache.end(); )
        it = active_goals.count(it->first) ? std::next(it) : ff_cache.erase(it);
}

const std::vector<Vector3>&                    get_waypoints()   { return waypoints; }
const std::unordered_map<uint64_t, FlowField>& get_flow_fields() { return ff_cache;  }
